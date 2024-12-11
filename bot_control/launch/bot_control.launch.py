from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    spawn_launch_path = PathJoinSubstitution(
        [FindPackageShare('bot_world'), 'launch', 'bot_world.launch.py']
    )

    # Launch reading_laser in a new terminal
    reading_laser_process = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'bot_control', 'reading_laser'],
        output='screen'
    )

    # Teleop keyboard node
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                spawn_launch_path
            ])
        ),
        teleop_keyboard,
        reading_laser_process,
    ])
