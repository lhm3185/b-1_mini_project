#!/usr/bin/env python3

import time
from pathlib import Path

import cv2
import rclpy
from interface_pkg.srv import WebcamDetection
from rclpy.node import Node
from ultralytics import YOLO


from ament_index_python.packages import (
    get_package_share_directory,
)


DEFAULT_MODEL_PATH = (
    Path(get_package_share_directory('vision_pkg'))
    / 'models'
    / 'webcam_yolo26n.pt'
)


class WebcamNode(Node):

    def __init__(self):
        super().__init__('webcam_node')

        # ROS 파라미터
        self.declare_parameter('camera_index', 0)
        self.declare_parameter(
            'model_path',
            str(DEFAULT_MODEL_PATH),
        )
        self.declare_parameter('target_class', 'car')
        self.declare_parameter(
            'confidence_threshold',
            0.85,
        )
        self.declare_parameter('consecutive_frames', 3)
        self.declare_parameter('show_window', True)

        self.camera_index = int(
            self.get_parameter('camera_index').value
        )
        self.model_path = Path(
            self.get_parameter('model_path').value
        ).expanduser()
        self.target_class = str(
            self.get_parameter('target_class').value
        ).strip().lower()
        self.confidence_threshold = float(
            self.get_parameter(
                'confidence_threshold'
            ).value
        )
        self.consecutive_frames = max(
            1,
            int(
                self.get_parameter(
                    'consecutive_frames'
                ).value
            ),
        )
        self.show_window = bool(
            self.get_parameter('show_window').value
        )

        # 모델 파일 확인
        if not self.model_path.is_file():
            raise FileNotFoundError(
                'YOLO 모델 파일을 찾을 수 없습니다: '
                f'{self.model_path}'
            )

        # YOLO 모델 로딩
        self.get_logger().info(
            f'YOLO 모델을 불러오는 중: {self.model_path}'
        )
        self.model = YOLO(str(self.model_path))

        if isinstance(self.model.names, dict):
            self.class_names = self.model.names
        else:
            self.class_names = dict(
                enumerate(self.model.names)
            )

        available_classes = {
            str(name).lower()
            for name in self.class_names.values()
        }

        if self.target_class not in available_classes:
            raise ValueError(
                f'모델에 "{self.target_class}" 클래스가 없습니다. '
                f'사용 가능한 클래스: '
                f'{sorted(available_classes)}'
            )

        # 웹캠 열기
        self.capture = cv2.VideoCapture(
            self.camera_index
        )

        if not self.capture.isOpened():
            raise RuntimeError(
                '웹캠을 열 수 없습니다: '
                f'camera_index={self.camera_index}'
            )

        # 웹캠 노드는 서비스 클라이언트입니다.
        self.detected_client = self.create_client(
            WebcamDetection,
            '/robot4/rc_car_detected',
        )

        # 검출 상태
        self.positive_streak = 0
        self.detection_confirmed = False
        self.detection_stamp = None

        # 서비스 요청 상태
        self.request_future = None
        self.request_started_at = None
        self.request_completed = False

        self.running = True

        # 웹캠 추론 주기: 0.2초
        self.camera_timer = self.create_timer(
            0.2,
            self.camera_callback,
        )

        # 서비스 미응답 시 재시도 주기: 1초
        self.retry_timer = self.create_timer(
            1.0,
            self.try_send_request,
        )

        self.get_logger().info(
            '웹캠 RC카 검출 노드를 시작합니다. '
            f'target={self.target_class}, '
            f'confidence={self.confidence_threshold}, '
            f'consecutive_frames={self.consecutive_frames}'
        )

    def camera_callback(self):
        frame_ok, frame = self.capture.read()
        frame_stamp = self.get_clock().now().to_msg()

        if not frame_ok:
            self.get_logger().warning(
                '웹캠 프레임을 읽지 못했습니다.'
            )

            if not self.detection_confirmed:
                self.positive_streak = 0

            return

        try:
            results = self.model.predict(
                source=frame,
                conf=self.confidence_threshold,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(
                f'YOLO 추론 실패: {exc}'
            )
            self.running = False
            return

        result = results[0]
        raw_detected = self.contains_target(result)

        # 아직 검출이 확정되지 않은 동안만
        # 연속 프레임을 계산합니다.
        if not self.detection_confirmed:
            if raw_detected:
                self.positive_streak += 1
            else:
                self.positive_streak = 0

            if (
                self.positive_streak
                >= self.consecutive_frames
            ):
                self.detection_confirmed = True
                self.detection_stamp = frame_stamp

                self.get_logger().info(
                    'RC카 연속 검출 완료. '
                    'AMR에 시작 요청을 보냅니다.'
                )

                self.try_send_request()

        if self.show_window:
            self.show_result(result)

    def contains_target(self, result):
        if (
            result.boxes is None
            or len(result.boxes) == 0
        ):
            return False

        class_ids = result.boxes.cls.cpu().tolist()

        for class_id in class_ids:
            class_name = str(
                self.class_names[int(class_id)]
            ).lower()

            if class_name == self.target_class:
                return True

        return False

    def try_send_request(self):
        if not self.detection_confirmed:
            return

        if self.request_completed:
            return

        # 이전 요청이 아직 처리 중인지 확인합니다.
        if self.request_future is not None:
            if self.request_future.done():
                return

            elapsed = (
                time.monotonic()
                - self.request_started_at
            )

            # 요청 후 1초가 지나지 않았다면 기다립니다.
            if elapsed < 1.0:
                return

            # 1초 동안 응답이 없으면 기존 요청을 취소하고
            # 다시 요청할 수 있도록 초기화합니다.
            old_future = self.request_future
            self.request_future = None
            self.request_started_at = None
            old_future.cancel()

            self.get_logger().warning(
                '서비스 응답이 없어 다시 시도합니다.'
            )

        # AMR의 서비스 서버가 아직 실행되지 않은 경우
        if not self.detected_client.service_is_ready():
            self.get_logger().warning(
                '/robot4/rc_car_detected '
                '서비스를 기다리는 중입니다.'
            )
            return

        request = WebcamDetection.Request()
        request.header.stamp = self.detection_stamp
        request.header.frame_id = 'webcam'
        request.detected = True

        self.get_logger().info(
            'AMR에 RC카 검출 요청을 보냅니다.'
        )

        self.request_started_at = time.monotonic()
        self.request_future = (
            self.detected_client.call_async(request)
        )
        self.request_future.add_done_callback(
            self.on_detection_response
        )

    def on_detection_response(self, future):
        # timeout으로 취소한 이전 요청의 콜백이면 무시합니다.
        if future.cancelled():
            return

        if future is not self.request_future:
            return

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'서비스 요청 실패: {exc}'
            )
            self.request_future = None
            self.request_started_at = None
            return

        self.request_future = None
        self.request_started_at = None
        self.request_completed = True

        if response.started:
            self.get_logger().info(
                'AMR이 검출 요청을 수락하고 '
                '동작을 시작했습니다.'
            )
        else:
            self.get_logger().warning(
                'AMR이 요청을 받았지만 '
                '동작을 시작하지 않았습니다.'
            )

        # 인터페이스 정의상 응답을 받으면
        # 같은 요청을 다시 보내지 않습니다.
        self.running = False

    def show_result(self, result):
        display_frame = result.plot()

        if self.request_completed:
            status_text = 'REQUEST COMPLETED'
            status_color = (255, 255, 0)

        elif self.detection_confirmed:
            status_text = 'WAITING FOR AMR'
            status_color = (0, 255, 255)

        else:
            status_text = (
                'SEARCHING '
                f'{self.positive_streak}/'
                f'{self.consecutive_frames}'
            )
            status_color = (0, 0, 255)

        cv2.putText(
            display_frame,
            status_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            status_color,
            2,
        )

        cv2.imshow(
            'Webcam RC Car Detection',
            display_frame,
        )

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False

    def close(self):
        if hasattr(self, 'capture'):
            self.capture.release()

        if self.show_window:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = WebcamNode()

        while rclpy.ok() and node.running:
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()