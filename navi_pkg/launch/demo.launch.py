"""한 PC 에서 전부 띄운다: navi.launch.py (위치 추정, Nav2, RViz, 주행 노드) + vision.launch.py (AMR 캠, 고정 웹캠).

  ros2 launch navi_pkg demo.launch.py
  ros2 launch navi_pkg demo.launch.py camera_index:=0     # 두 런치의 인자를 그대로 쓸 수 있다

고정 웹캠이 이 PC 에 꽂혀 있어야 한다. 로봇이 세 번 울린 뒤에 RC카를 웹캠 시야에 넣는다.
웹캠 노드는 응답을 받으면 끝난다. 다시 시연하려면 다른 터미널에서 ros2 run vision_pkg webcam_node.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package, launch_file):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', launch_file)))


def generate_launch_description():
    return LaunchDescription([
        include('navi_pkg', 'navi.launch.py'),
        include('vision_pkg', 'vision.launch.py'),
    ])
