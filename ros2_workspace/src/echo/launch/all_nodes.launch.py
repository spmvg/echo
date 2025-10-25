from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    stt_onboard_node = Node(
        package='echo',
        executable='stt_onboard',
        name='stt_onboard',
        output='screen'
    )

    tts_onboard_node = Node(
        package='echo',
        executable='tts_onboard',
        name='tts_onboard',
        output='screen'
    )

    return LaunchDescription([
        stt_onboard_node,
        tts_onboard_node
    ])
