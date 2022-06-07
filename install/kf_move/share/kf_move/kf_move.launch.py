from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='kf_move', node_executable='kf_move', output='screen'),
    ])
    
