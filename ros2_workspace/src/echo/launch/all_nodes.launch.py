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

    # rosbridge exposes selected ROS 2 topics over WebSocket (port 9090).
    # Only the topics listed in topics_glob are accessible to external clients.
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            # Whitelist: only expose the remote listening-control topics.
            # rosbridge parses this as: strip outer [], split on comma, strip single quotes.
            'topics_glob': "['/stt_onboard/set_listening','/stt_onboard/listening_state']",
        }],
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
