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

    # rosbridge exposes all ROS 2 topics/services over WebSocket (port 9090)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
    )

    initialization_node = Node(
        package='echo',
        executable='initialization',
        name='initialization',
        output='screen'
    )

    return LaunchDescription([
        stt_onboard_node,
        tts_onboard_node,
        rosbridge_node,
        initialization_node,
    ])
