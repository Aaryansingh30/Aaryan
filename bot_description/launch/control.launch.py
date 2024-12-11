from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node



def generate_launch_description():
    # Path to spawn.launch.py
    spawn_launch_path = PathJoinSubstitution(
        [FindPackageShare('bot_description'), 'launch', 'spawn.launch.py']
    )
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch_path)
        ),

        teleop_keyboard,
    ])
