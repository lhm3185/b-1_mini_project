"""웹캠 검출 신호를 받으면 출발해 RC카를 찾고, 일정 거리를 유지하며 따라간다.

실행 (Nav2·RViz 까지 한 번에):
  ros2 launch navi_pkg navi.launch.py
노드만:
  ros2 run navi_pkg follow_car --ros-args -r __ns:=/robot4

흐름: 대기 -> (서비스 rc_car_detected) -> init_car (undock, 초기 위치) -> 지정 좌표 이동
      -> detecting_motion (회전 탐색) -> (토픽 rc_car_target) -> follow (추종) -> home_pose (복귀, dock) -> 대기
추종을 끝내고 dock 시키려면 다른 터미널에서:
  ros2 service call /robot4/stop_follow std_srvs/srv/Trigger
강의 코드 day3/3_1_a_nav_to_pose.py 의 TurtleBot4Navigator 사용법을 따른다.
"""
import math
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from interface_pkg.msg import AmrState, RcCarTarget
from interface_pkg.srv import WebcamDetection
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

DISTANCE_TOLERANCE = 0.05   # 유지 거리보다 이만큼 넘게 멀 때만 전진한다 (m)
ANGLE_TOLERANCE = 0.05      # RC카 방향이 이만큼 이내면 회전하지 않는다 (rad)
LOST_TIMEOUT = 2.0          # rc_car_target 이 이만큼 안 오면 놓친 것 (주행 중에는 0.7~1.0 초 간격으로 온다)
FRESH_TIMEOUT = 0.3         # 이보다 오래된 rc_car_target 으로는 회전하지 않는다 (늦은 값으로 돌면 좌우로 떨린다)
SCAN_SPEED = 0.4            # 탐색 회전 명령 (rad/s). 실제로는 약 0.22 rad/s 로 돈다 (한 바퀴 약 28 초)
SCAN_TIMEOUT = 60.0         # 이 시간 동안 못 찾으면 포기 (약 두 바퀴)
MAX_LINEAR_SPEED = 0.2      # 최대 전진 속도 (m/s). 후진은 하지 않는다
MAX_ANGULAR_SPEED = 0.5     # 최대 회전 속도 (rad/s)
LINEAR_GAIN = 0.6
ANGULAR_GAIN = 0.6
NAV2_RETRY = 20.0           # 다시 기동시킨 Nav2 가 이 시간 안에 켜지지 않으면 한 번 더 기동한다 (초)


class FollowCar:

    def __init__(self):
        self.navigator = TurtleBot4Navigator()
        declare = self.navigator.declare_parameter
        self.keep_distance = declare('keep_distance', 0.8).value    # 로봇 중심 기준 (m). 0.64 m 보다 가까우면 깊이가 안 나온다
        self.follow_sec = declare('follow_sec', 600.0).value        # 출발부터 이 시간이 지나면 복귀한다
        self.go_to_goal = declare('go_to_goal', True).value         # False 면 undock 한 자리에서 바로 탐색
        self.goal = [declare('goal_x', -3.88).value, declare('goal_y', -2.84).value]
        self.dock_front = [declare('dock_front_x', -1.0).value, declare('dock_front_y', -0.07).value]

        self.state = AmrState.IDLE
        self.start_requested = False
        self.deadline = 0.0         # 이 시각이 지나면 탐색·추종을 끝내고 복귀한다
        self.target = None          # 마지막 rc_car_target
        self.target_time = 0.0      # 그것을 받은 시각 (이 PC 의 시계)

        self.cmd_pub = self.navigator.create_publisher(Twist, 'cmd_vel_unstamped', 10)
        self.state_pub = self.navigator.create_publisher(AmrState, 'amr_state', 10)
        self.audio_pub = self.navigator.create_publisher(AudioNoteVector, 'cmd_audio', 10)
        self.navigator.create_subscription(RcCarTarget, 'rc_car_target', self.on_target, 10)
        self.navigator.create_service(WebcamDetection, 'rc_car_detected', self.on_detected)
        self.navigator.create_service(Trigger, 'stop_follow', self.on_stop)
        self.nav2_state = self.navigator.create_client(GetState, 'bt_navigator/get_state')
        self.planner_state = self.navigator.create_client(GetState, 'planner_server/get_state')
        self.nav2_manager = self.navigator.create_client(
            ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes')
        self.navigator.create_timer(1.0, lambda: self.state_pub.publish(AmrState(state=self.state)))

    def on_detected(self, request, response):
        """웹캠이 RC카를 봤다. 대기 중일 때만 출발한다. 여기서는 표시만 하고 바로 답한다."""
        response.started = request.detected and self.state == AmrState.IDLE and not self.start_requested
        if response.started:
            self.start_requested = True
            self.navigator.info('웹캠 검출 신호 수신, 출발합니다')
        else:
            self.navigator.warn('웹캠 검출 요청을 무시합니다 (미검출 또는 이미 동작 중)')
        return response

    def on_stop(self, request, response):
        """사용자가 중단을 요청했다. 기한을 지금으로 당겨 복귀·dock 하게 한다."""
        self.deadline = 0.0
        self.beep(1320, 880)
        response.success = True
        response.message = '추종을 끝내고 dock 으로 돌아간다'
        self.navigator.info('중단 요청 수신')
        return response

    def on_target(self, msg):
        self.target = msg
        self.target_time = time.monotonic()

    def beep(self, *frequencies):
        """알림음. 대기 880-880-880(세 번), 출발 880-1320, 중단 1320-880."""
        note = Duration(nanosec=200_000_000)
        self.audio_pub.publish(AudioNoteVector(
            notes=[AudioNote(frequency=f, max_runtime=note) for f in frequencies]))

    def wait_ready_beep(self):
        """출발 신호를 받을 준비가 됐음을 세 번 울려 알린다. 로봇의 스피커가 연결될 때까지 잠깐 기다린다."""
        start = time.monotonic()
        while rclpy.ok() and self.audio_pub.get_subscription_count() == 0 and time.monotonic() - start < 10.0:
            rclpy.spin_once(self.navigator, timeout_sec=0.1)
        self.beep(880, 0, 880, 0, 880)      # 0 Hz 는 쉼표

    def set_state(self, state):
        self.state = state
        self.state_pub.publish(AmrState(state=state))
        self.navigator.info(f'상태: {state}')

    def stop(self):
        self.cmd_pub.publish(Twist())

    def init_car(self):
        """도킹 상태에서 시작해 undock 하고 초기 위치를 잡는다."""
        if not self.navigator.getDockedStatus():
            self.navigator.info('언독 돼있음 독 먼저')
            self.navigator.dock()
        self.navigator.undock()
        self.navigator.info('언독 완료')

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.SOUTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.info('0.0, 0.0 초기 포즈 설정 완료')
        self.start_nav2()

    def lifecycle_state(self, client):
        """Nav2 노드의 상태 이름('active', 'inactive' 등). 답이 없으면 None."""
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self.navigator, future, timeout_sec=2.0)
        return future.result().current_state.label if future.done() and future.result() else None

    def start_nav2(self):
        """Nav2 가 켜질 때까지 기다린다. 기동을 포기한 상태면 바로 다시 기동시킨다.

        도킹 중에는 절전으로 라이다가 꺼져 있어 지도 위 위치(map TF)가 나오지 않는다.
        Nav2 는 그 위치를 60 초 기다리다 planner_server 를 켜지 못한 채(inactive) 기동을 포기하므로,
        undock 하고 초기 위치를 준 지금 다시 켠다. 기동 중(activating)이면 건드리지 않고 기다린다.
        """
        self.navigator.waitUntilNav2Active(navigator='amcl')    # 위치 추정이 초기 위치를 받을 때까지
        restarted = None
        while rclpy.ok() and self.lifecycle_state(self.nav2_state) != 'active':
            gave_up = self.lifecycle_state(self.planner_state) == 'inactive'
            if gave_up and (restarted is None or time.monotonic() - restarted > NAV2_RETRY):
                self.navigator.info('Nav2 가 꺼져 있어 다시 기동한다')
                for command in (ManageLifecycleNodes.Request.RESET, ManageLifecycleNodes.Request.STARTUP):
                    future = self.nav2_manager.call_async(ManageLifecycleNodes.Request(command=command))
                    rclpy.spin_until_future_complete(self.navigator, future)
                restarted = time.monotonic()
            rclpy.spin_once(self.navigator, timeout_sec=0.5)
        self.navigator.info('Nav2 준비 완료')

    def detecting_motion(self, angular_speed=SCAN_SPEED):
        """RC카 타깃 메시지를 받을 때까지 회전하며 탐색한다. 찾으면 True."""
        self.set_state(AmrState.SCANNING)
        cmd = Twist()
        cmd.angular.z = angular_speed
        start = time.monotonic()    # 탐색을 시작한 뒤에 받은 메시지만 쓴다
        while rclpy.ok() and time.monotonic() < min(self.deadline, start + SCAN_TIMEOUT):
            if self.target_time > start:
                self.stop()
                return True
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.navigator, timeout_sec=0.05)
        self.stop()
        return False

    def follow(self):
        """유지 거리를 지키며 RC카를 따라간다. 놓치면 돌아온다."""
        self.set_state(AmrState.FOLLOWING)
        while rclpy.ok() and time.monotonic() < self.deadline:
            rclpy.spin_once(self.navigator, timeout_sec=0.05)
            age = time.monotonic() - self.target_time
            if age > LOST_TIMEOUT:
                self.navigator.info('RC카를 놓침')
                break

            cmd = Twist()
            distance_error = self.target.distance - self.keep_distance
            if distance_error > DISTANCE_TOLERANCE:     # 멀 때만 전진한다. RC카가 멈추면 AMR 도 멈춘다
                cmd.linear.x = min(MAX_LINEAR_SPEED, LINEAR_GAIN * distance_error)
            angle = math.atan2(self.target.offset, self.target.distance)
            if abs(angle) > ANGLE_TOLERANCE and age < FRESH_TIMEOUT:
                cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, ANGULAR_GAIN * angle))
            self.cmd_pub.publish(cmd)
        self.stop()

    def home_pose(self):
        """dock 앞으로 돌아와 dock 한다."""
        self.set_state(AmrState.MOVING)
        dock_pose = self.navigator.getPoseStamped(self.dock_front, TurtleBot4Directions.NORTH)
        self.navigator.startToPose(dock_pose)
        self.navigator.dock()
        self.navigator.info('도킹 완료')

    def run(self):
        """출발 신호 한 번에 대한 전체 순서."""
        self.deadline = time.monotonic() + self.follow_sec
        self.beep(880, 1320)
        self.set_state(AmrState.MOVING)
        self.init_car()
        if self.go_to_goal:
            goal_pose = self.navigator.getPoseStamped(self.goal, TurtleBot4Directions.WEST)
            self.navigator.startToPose(goal_pose)

        while rclpy.ok() and time.monotonic() < self.deadline and self.detecting_motion():
            self.follow()

        self.home_pose()
        self.start_requested = False
        self.set_state(AmrState.IDLE)


def main():
    rclpy.init()
    follow_car = FollowCar()
    try:
        while rclpy.ok():       # 한 번 끝나면 다음 출발 신호를 다시 기다린다
            follow_car.wait_ready_beep()
            follow_car.navigator.info('웹캠 검출 신호 대기 (서비스 rc_car_detected)')
            while rclpy.ok() and not follow_car.start_requested:
                rclpy.spin_once(follow_car.navigator, timeout_sec=0.1)
            if follow_car.start_requested:
                follow_car.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            follow_car.stop()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
