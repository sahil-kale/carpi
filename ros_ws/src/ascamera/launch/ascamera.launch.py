from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "ascamera",
            package='ascamera',
            executable='ascamera_node',
            respawn=False,
            output='both',
            parameters=[
                {"confiPath": "./ascamera/configurationfiles"}
            ]
        ),
    ])
