"""웹캠 검출 신호를 받으면 출발해 RC카를 찾고, 일정 거리를 유지하며 따라간다.

실행:
  ros2 run navi_pkg rc_car_follower --ros-args -r __ns:=/robot4

흐름: 대기 -> (서비스 rc_car_detected) -> undock -> 지정 좌표 이동 -> 회전하며 탐색
      -> (토픽 rc_car_target) -> 추종 (멀면 전진, 가까우면 제자리) -> dock 앞으로 복귀 -> dock
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
import rclpy
from std_srvs.srv import Trigger
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

SCAN_SPEED = 0.4        # 탐색 회전 명령 (rad/s). 실제로는 약 0.22 rad/s 로 돈다 (한 바퀴 약 28 초)
SCAN_TIMEOUT = 60.0     # 이 시간 동안 못 찾으면 포기 (약 두 바퀴)
LOST_SEC = 2.0          # rc_car_target 이 이만큼 안 오면 놓친 것 (주행 중에는 0.7~1.0 초 간격으로 온다)
DIST_TOL = 0.05         # 유지 거리보다 이만큼 넘게 멀 때만 전진한다 (m)
ANGLE_TOL = 0.05        # RC카 방향이 이만큼 이내면 회전하지 않는다 (rad)
K_LIN, MAX_LIN = 0.6, 0.2       # 전진 이득, 최대 전진 속도 (m/s)
K_ANG, MAX_ANG = 0.6, 0.5       # 회전 이득, 최대 회전 속도 (rad/s)


class RcCarFollower:

    def __init__(self):
        self.nav = TurtleBot4Navigator()
        declare = self.nav.declare_parameter
        self.keep_distance = declare('keep_distance', 0.8).value    # 로봇 중심 기준 (m). 0.64 m 보다 가까우면 깊이가 안 나온다
        self.follow_sec = declare('follow_sec', 60.0).value         # 출발부터 이 시간이 지나면 복귀한다
        self.go_to_goal = declare('go_to_goal', True).value         # False 면 undock 한 자리에서 바로 탐색
        self.goal = [declare('goal_x', -3.88).value, declare('goal_y', -2.84).value]
        self.dock_front = [declare('dock_front_x', -1.0).value, declare('dock_front_y', -0.07).value]

        self.state = AmrState.IDLE
        self.start_requested = False
        self.deadline = 0.0         # 이 시각이 지나면 탐색·추종을 끝내고 복귀한다
        self.target = None          # 마지막 rc_car_target
        self.target_time = 0.0      # 그것을 받은 시각 (이 PC 의 시계)

        self.cmd_pub = self.nav.create_publisher(Twist, 'cmd_vel_unstamped', 10)
        self.state_pub = self.nav.create_publisher(AmrState, 'amr_state', 10)
        self.audio_pub = self.nav.create_publisher(AudioNoteVector, 'cmd_audio', 10)
        self.nav.create_subscription(RcCarTarget, 'rc_car_target', self.on_target, 10)
        self.nav.create_service(WebcamDetection, 'rc_car_detected', self.on_detected)
        self.nav.create_service(Trigger, 'stop_follow', self.on_stop)
        self.nav.create_timer(1.0, lambda: self.state_pub.publish(AmrState(state=self.state)))

    def on_detected(self, request, response):
        """웹캠이 RC카를 봤다. 대기 중일 때만 출발한다. 여기서는 표시만 하고 바로 답한다."""
        response.started = request.detected and self.state == AmrState.IDLE and not self.start_requested
        if response.started:
            self.start_requested = True
            self.nav.info('웹캠 검출 신호 수신, 출발')
        return response

    def on_stop(self, request, response):
        """사용자가 중단을 요청했다. 기한을 지금으로 당겨 복귀·dock 하게 한다."""
        self.deadline = 0.0
        response.success = True
        response.message = '추종을 끝내고 dock 으로 돌아간다'
        self.nav.info('중단 요청 수신')
        return response

    def beep(self):
        note = Duration(nanosec=200_000_000)
        self.audio_pub.publish(AudioNoteVector(notes=[
            AudioNote(frequency=880, max_runtime=note), AudioNote(frequency=1320, max_runtime=note)]))

    def on_target(self, msg):
        self.target = msg
        self.target_time = time.monotonic()

    def set_state(self, state):
        self.state = state
        self.state_pub.publish(AmrState(state=state))
        self.nav.info(f'상태: {state}')

    def stop(self):
        self.cmd_pub.publish(Twist())

    def scan(self):
        """RC카가 보일 때까지 제자리에서 돈다. 찾으면 True."""
        self.set_state(AmrState.SCANNING)
        cmd = Twist()
        cmd.angular.z = SCAN_SPEED
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() < min(self.deadline, start + SCAN_TIMEOUT):
            if self.target_time > start:
                self.stop()
                return True
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.nav, timeout_sec=0.05)
        self.stop()
        return False

    def follow(self):
        """유지 거리를 지키며 RC카를 따라간다. 놓치면 돌아온다."""
        self.set_state(AmrState.FOLLOWING)
        while rclpy.ok() and time.monotonic() < self.deadline:
            rclpy.spin_once(self.nav, timeout_sec=0.05)
            if time.monotonic() - self.target_time > LOST_SEC:
                self.nav.info('RC카를 놓침')
                break
            cmd = Twist()
            error = self.target.distance - self.keep_distance
            if error > DIST_TOL:        # 멀 때만 전진한다. 후진 명령은 로봇이 통째로 무시해 쓰지 않는다
                cmd.linear.x = min(MAX_LIN, K_LIN * error)
            angle = math.atan2(self.target.offset, self.target.distance)
            if abs(angle) > ANGLE_TOL:
                cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * angle))
            self.cmd_pub.publish(cmd)
        self.stop()

    def run(self):
        if not self.nav.getDockedStatus():
            self.nav.info('도킹 상태에서 시작한다')
            self.nav.dock()

        self.nav.info('웹캠 검출 신호 대기 (서비스 rc_car_detected)')
        while rclpy.ok() and not self.start_requested:
            rclpy.spin_once(self.nav, timeout_sec=0.05)
        self.deadline = time.monotonic() + self.follow_sec
        self.beep()

        self.set_state(AmrState.MOVING)
        self.nav.undock()
        self.nav.setInitialPose(self.nav.getPoseStamped([0.0, 0.0], TurtleBot4Directions.SOUTH))
        self.nav.waitUntilNav2Active()
        if self.go_to_goal:
            self.nav.startToPose(self.nav.getPoseStamped(self.goal, TurtleBot4Directions.WEST))

        while rclpy.ok() and self.scan():
            self.follow()

        self.set_state(AmrState.MOVING)
        self.nav.startToPose(self.nav.getPoseStamped(self.dock_front, TurtleBot4Directions.NORTH))
        self.nav.dock()
        self.start_requested = False
        self.set_state(AmrState.IDLE)


def main():
    rclpy.init()
    follower = RcCarFollower()
    try:
        follower.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            follower.stop()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
