"""AMR 쪽을 한 번에 띄운다: 위치 추정, Nav2, RViz, 주행 노드(follow_car).

  ros2 launch navi_pkg navi.launch.py
  ros2 launch navi_pkg navi.launch.py map:=/경로/지도.yaml rviz:=false
  ros2 launch navi_pkg navi.launch.py nav2:=false       # Nav2 가 이미 떠 있을 때 주행 노드만

로봇은 도킹 상태에서 시작한다. 추종을 끝내려면:
  ros2 service call /robot4/stop_follow std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, launch_file, arguments, condition):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=arguments.items(), condition=IfCondition(condition))


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    nav2 = LaunchConfiguration('nav2')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot4'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/my_map.yaml')),
        DeclareLaunchArgument('nav2', default_value='true', description='위치 추정과 Nav2 도 띄운다'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('follow_sec', default_value='600.0', description='출발부터 이 시간이 지나면 복귀 (초)'),
        include('turtlebot4_navigation', 'localization.launch.py',
                {'namespace': namespace, 'map': LaunchConfiguration('map')}, nav2),
        include('turtlebot4_navigation', 'nav2.launch.py', {'namespace': namespace}, nav2),
        include('turtlebot4_viz', 'view_navigation.launch.py', {'namespace': namespace}, LaunchConfiguration('rviz')),
        Node(
            package='navi_pkg', executable='follow_car', namespace=namespace, output='screen',
            parameters=[{'follow_sec': ParameterValue(LaunchConfiguration('follow_sec'), value_type=float)}],
        ),
    ])
