"""AMR 카메라(OAK-D)로 RC카를 찾고, depth 로 거리·방향을 계산해 서비스로 알려 준다.

실행 (TF 리매핑이 없으면 base_link 를 찾지 못한다):
  ros2 run vision_pkg amr_cam --ros-args -r __ns:=/robot4 \
    -r /tf:=/robot4/tf -r /tf_static:=/robot4/tf_static -p model_path:=<모델.pt>

강의 코드 day3/3_3_d_depth_to_nav_goal_ts.py 의 방식(RGB+depth 동기, 픽셀+깊이 -> 카메라 좌표 -> TF)을 따른다.
"""
import cv2
from geometry_msgs.msg import PointStamped
from interface_pkg.srv import RcCarTarget
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO

LOW_CONF = 0.5                # 이 값 이상은 모두 받아 디버그 영상·로그에 쓴다
DEPTH_MIN, DEPTH_MAX = 0.2, 5.0   # 유효 깊이 (m)
MIN_VALID_RATIO = 0.1         # 박스 가운데 영역에서 유효 깊이 픽셀의 최소 비율


class AmrCam(Node):

    def __init__(self):
        super().__init__('amr_cam')
        self.conf = self.declare_parameter('conf', 0.85).value
        self.hold_sec = self.declare_parameter('hold_sec', 1.0).value
        self.car_height = self.declare_parameter('car_height', 0.0).value   # RC카 높이 (m), 0 이면 비교 로그 없음
        best_effort = self.declare_parameter('best_effort', False).value
        model_path = self.declare_parameter('model_path', '').value

        self.model = YOLO(model_path)
        self.car_id = next(i for i, name in self.model.names.items() if name == 'car')
        self.model.predict(np.zeros((480, 640, 3), np.uint8), verbose=False)   # 첫 추론이 느리므로 미리 한 번

        self.pair = None        # 아직 처리하지 않은 (RGB, depth) 한 쌍
        self.K = None           # 카메라 내부 파라미터
        self.tf = None          # 카메라 -> base_link 변환 (고정이라 한 번만 받는다)
        self.latest = None      # (응답, 처리한 시각)
        self.has_image = False
        self.last_image = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = qos_profile_sensor_data if best_effort else 10
        rgb_sub = Subscriber(self, CompressedImage, 'oakd/rgb/image_raw/compressed', qos_profile=qos)
        depth_sub = Subscriber(self, Image, 'oakd/stereo/image_raw', qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.on_pair)
        self.create_subscription(CameraInfo, 'oakd/rgb/camera_info', self.on_camera_info, 1)

        self.debug_pub = self.create_publisher(CompressedImage, 'rc_car_debug/compressed', 1)
        self.create_service(RcCarTarget, 'rc_car_target', self.on_request)
        self.create_timer(0.1, self.process)
        self.get_logger().info(f'amr_cam 시작: conf={self.conf}, hold_sec={self.hold_sec}')

    def on_pair(self, rgb_msg, depth_msg):
        self.pair = (rgb_msg, depth_msg)

    def on_camera_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def on_request(self, request, response):
        """저장해 둔 최신 결과를 돌려준다. hold_sec 보다 오래됐으면 놓친 것이다."""
        if self.latest is not None:
            result, processed_at = self.latest
            if (self.get_clock().now() - processed_at).nanoseconds < self.hold_sec * 1e9:
                return result
        return response     # detected = False

    def process(self):
        now = self.get_clock().now()
        if self.pair is None:
            if (now - self.last_image).nanoseconds > 2e9:
                self.has_image = False
                self.get_logger().warn('영상 없음 (도킹 중이면 카메라가 꺼져 있다)', throttle_duration_sec=5.0)
            return
        self.last_image = now
        if not self.has_image:
            self.get_logger().info('영상 수신 중')
            self.has_image = True
        (rgb_msg, depth_msg), self.pair = self.pair, None    # 같은 쌍을 두 번 쓰지 않는다

        if self.K is None:
            return
        if self.tf is None:
            try:
                self.tf = self.tf_buffer.lookup_transform('base_link', depth_msg.header.frame_id, Time())
            except TransformException as e:
                self.get_logger().warn(f'TF 대기 중: {e}', throttle_duration_sec=5.0)
                return

        rgb = cv2.imdecode(np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)
        depth = np.frombuffer(depth_msg.data, np.uint16).reshape(depth_msg.height, depth_msg.width)   # 16UC1, mm

        boxes = self.model.predict(rgb, conf=LOW_CONF, verbose=False)[0].boxes
        car = None      # 기준을 넘는 car 중 신뢰도가 가장 높은 박스
        for box in boxes:
            conf, is_car = float(box.conf[0]), int(box.cls[0]) == self.car_id
            if is_car and conf >= self.conf and (car is None or conf > float(car.conf[0])):
                car = box
            elif is_car:
                self.get_logger().debug(f'기준 미달 car: conf={conf:.2f}')

        target = self.locate(car, depth, rgb_msg.header.stamp) if car is not None else None
        if target is not None:
            self.latest = (target, now)
        if self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(rgb, boxes, target, rgb_msg.header)

    def locate(self, car, depth, stamp):
        """박스 가운데 50% 영역의 깊이 중앙값으로 base_link 기준 거리·좌우를 구한다."""
        x1, y1, x2, y2 = (int(v) for v in car.xyxy[0])
        w, h = x2 - x1, y2 - y1
        z = depth[y1 + h // 4:y2 - h // 4, x1 + w // 4:x2 - w // 4] / 1000.0
        valid = z[(z > DEPTH_MIN) & (z < DEPTH_MAX)]
        if valid.size <= MIN_VALID_RATIO * z.size:
            return None
        z = float(np.median(valid))

        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        u, v = (x1 + x2) / 2, (y1 + y2) / 2
        pt = PointStamped()
        pt.point.x, pt.point.y, pt.point.z = (u - cx) * z / fx, (v - cy) * z / fy, z
        pt = do_transform_point(pt, self.tf)

        target = RcCarTarget.Response(detected=True, distance=pt.point.x, offset=pt.point.y)
        target.header.stamp = stamp
        target.header.frame_id = 'base_link'

        log = f'car conf={float(car.conf[0]):.2f} distance={target.distance:.2f} offset={target.offset:+.2f}'
        if self.car_height > 0:     # 박스 높이로 구한 카메라 기준 거리 (depth 값과 비교용)
            log += f' | depth z={z:.2f} box z={fy * self.car_height / h:.2f}'
        self.get_logger().info(log, throttle_duration_sec=1.0)
        return target

    def publish_debug(self, rgb, boxes, target, header):
        for box in boxes:
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            is_car = int(box.cls[0]) == self.car_id
            color = (0, 255, 0) if is_car and float(box.conf[0]) >= self.conf else (0, 0, 255)
            cv2.rectangle(rgb, (x1, y1), (x2, y2), color, 2)
            cv2.putText(rgb, f'{self.model.names[int(box.cls[0])]} {float(box.conf[0]):.2f}', (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        if target is not None:
            cv2.putText(rgb, f'{target.distance:.2f} m  {target.offset:+.2f} m', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        msg = CompressedImage(header=header, format='jpeg')
        msg.data = cv2.imencode('.jpg', rgb)[1].tobytes()
        self.debug_pub.publish(msg)


def main():
    rclpy.init()
    node = AmrCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
