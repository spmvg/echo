from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    stt_onboard_node = Node(
        package='echo',
        executable='stt_onboard',
        name='stt_onboard',
        output='screen'
    )

    speech_logger_node = Node(
        package='echo',
        executable='speech_logger',
        name='speech_logger',
        output='screen'
    )

    return LaunchDescription([
        stt_onboard_node,
        speech_logger_node
    ])
