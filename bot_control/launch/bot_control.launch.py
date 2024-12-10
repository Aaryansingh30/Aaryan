import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = "bot_description"
    world_name = "bot_world"
    robot_name = "AaryanSingh_Bot"

    # Path to the robot's URDF
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'bot.urdf.xacro'
    )

    # Convert xacro to XML
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Launch RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rviz.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch Gazebo 
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': os.path.join(get_package_share_directory(world_name), 'worlds', 'bot_world.world')}.items()
    )

    # Spawn robot into Gazebo 
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # Run the reading_laser in a new terminal
    reading_laser_process = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'bot_control', 'reading_laser'],
        output='screen'
    )

    #Run the teleop_keyboard in new terminal
    teleop_keyboard =  ExecuteProcess(
            cmd=['gnome-terminal', '--','ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            ],
            output='screen'
        )

    return LaunchDescription([
        rviz_launch,
        gazebo_launch,
        spawn_entity,
        reading_laser_process,
        teleop_keyboard,
    ])
