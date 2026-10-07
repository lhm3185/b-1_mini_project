"""비전 노드 두 개를 띄운다: AMR 캠(amr_cam)과 고정 웹캠(webcam_node).

  ros2 launch vision_pkg vision.launch.py
  ros2 launch vision_pkg vision.launch.py webcam:=false          # 웹캠이 다른 PC 에 있을 때
  ros2 launch vision_pkg vision.launch.py camera_index:=0

두 노드 모두 검출 화면을 창으로 띄운다. 웹캠 노드는 AMR 의 응답을 받으면 스스로 끝난다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot4'),
        DeclareLaunchArgument('webcam', default_value='true', description='고정 웹캠 노드도 띄운다'),
        DeclareLaunchArgument('camera_index', default_value='2', description='고정 웹캠의 /dev/video 번호'),
        Node(
            package='vision_pkg', executable='amr_cam', namespace=namespace, output='screen',
            parameters=[{'show_window': True}],
            # 로봇의 TF 는 /robot4/tf 로 나온다. 이 리매핑이 없으면 base_link 를 찾지 못한다.
            remappings=[('/tf', [namespace, '/tf']), ('/tf_static', [namespace, '/tf_static'])],
        ),
        Node(
            package='vision_pkg', executable='webcam_node', output='screen',
            parameters=[{'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int)}],
            condition=IfCondition(LaunchConfiguration('webcam')),
        ),
    ])
