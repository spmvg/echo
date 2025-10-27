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

    initialization_node = Node(
        package='echo',
        executable='initialization',
        name='initialization',
        output='screen'
    )

    sounds_node = Node(
        package='echo',
        executable='sound_player',
        name='sound_player',
        output='screen'
    )

    return LaunchDescription([
        stt_onboard_node,
        tts_onboard_node,
        # initialization_node,
        sounds_node,
    ])
