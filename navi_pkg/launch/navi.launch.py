"""AMR 쪽을 한 번에 띄운다: 위치 추정, Nav2, RViz, 주행 노드(follow_car).

  ros2 launch navi_pkg navi.launch.py
  ros2 launch navi_pkg navi.launch.py map:=/경로/지도.yaml rviz:=false
  ros2 launch navi_pkg navi.launch.py nav2:=false       # Nav2 가 이미 떠 있을 때 주행 노드만
  ros2 launch navi_pkg navi.launch.py use_nav2:=false   # 추종을 속도 명령 직접으로 (10/7 시연 방식)
  ros2 launch navi_pkg navi.launch.py dummy:=false      # 더미를 장애물로 넣지 않는다 (Nav2 기본 설정 그대로)

로봇은 도킹 상태에서 시작한다. 추종을 끝내려면:
  ros2 service call /robot4/stop_follow std_srvs/srv/Trigger
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def include(package, launch_file, arguments, condition):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments=arguments.items(), condition=IfCondition(condition))


def nav2_with_dummy(context):
    """Nav2 를 띄운다. dummy 가 true 면 costmap 이 AMR 캠이 본 더미(dummy_points)도 장애물로 받게 한다.

    더미는 라이다보다 낮아 Nav2 가 스스로는 보지 못한다. TurtleBot4 의 nav2.yaml 을 읽어
    costmap 의 관측 소스에 dummy 를 더한 사본을 만들어 쓴다(원본은 그대로).
    """
    namespace = LaunchConfiguration('namespace').perform(context)
    arguments = {'namespace': namespace}
    if LaunchConfiguration('dummy').perform(context) == 'true':
        source = os.path.join(get_package_share_directory('turtlebot4_navigation'), 'config', 'nav2.yaml')
        with open(source) as f:
            params = yaml.safe_load(f)

        def add_dummy(node):
            for key, value in node.items():
                if not isinstance(value, dict):
                    continue
                if key in ('voxel_layer', 'obstacle_layer') and 'observation_sources' in value:
                    value['observation_sources'] += ' dummy'
                    value['dummy'] = {
                        'topic': namespace + '/dummy_points', 'data_type': 'PointCloud2',
                        'marking': True, 'clearing': False,
                        'min_obstacle_height': 0.0, 'max_obstacle_height': 2.0,
                        'obstacle_min_range': 0.0, 'obstacle_max_range': 2.5,
                    }
                else:
                    add_dummy(value)
        add_dummy(params)
        arguments['params_file'] = os.path.join(tempfile.gettempdir(), 'nav2_with_dummy.yaml')
        with open(arguments['params_file'], 'w') as f:
            yaml.safe_dump(params, f)
    return [include('turtlebot4_navigation', 'nav2.launch.py', arguments, LaunchConfiguration('nav2'))]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    nav2 = LaunchConfiguration('nav2')
    return LaunchDescription([
        # 사람 터미널에서는 디스커버리 설정이 ROS_SUPER_CLIENT=True 가 된다. 노드 여럿이 한꺼번에 그렇게 뜨면
        # 서로를 찾는 것이 느려져 Nav2 기동이 멈춘다(10/7 실측). 노드끼리의 통신에는 필요 없으므로 끈다.
        SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
        DeclareLaunchArgument('namespace', default_value='/robot4'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/my_map.yaml')),
        DeclareLaunchArgument('nav2', default_value='true', description='위치 추정과 Nav2 도 띄운다'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('follow_sec', default_value='600.0', description='출발부터 이 시간이 지나면 복귀 (초)'),
        DeclareLaunchArgument('use_nav2', default_value='true', description='추종을 Nav2 에 맡긴다. false 면 속도 명령 직접'),
        DeclareLaunchArgument('dummy', default_value='true', description='AMR 캠이 본 더미를 Nav2 장애물로 넣는다'),
        include('turtlebot4_navigation', 'localization.launch.py',
                {'namespace': namespace, 'map': LaunchConfiguration('map')}, nav2),
        OpaqueFunction(function=nav2_with_dummy),
        include('turtlebot4_viz', 'view_navigation.launch.py', {'namespace': namespace}, LaunchConfiguration('rviz')),
        Node(
            package='navi_pkg', executable='follow_car', namespace=namespace, output='screen',
            parameters=[{
                'follow_sec': ParameterValue(LaunchConfiguration('follow_sec'), value_type=float),
                'use_nav2': ParameterValue(LaunchConfiguration('use_nav2'), value_type=bool),
            }],
            # Nav2 추종은 지도 위 로봇 위치(TF)가 필요하다. 로봇의 TF 는 /robot4/tf 로 나온다.
            remappings=[('/tf', [namespace, '/tf']), ('/tf_static', [namespace, '/tf_static'])],
        ),
    ])
