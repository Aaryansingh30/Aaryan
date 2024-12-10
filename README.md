To launch the rviz of the robot run the command: ros2 launch bot_description rviz.launch.py

To spawn the robot in gazebo and rviz in an empty world : ros2 launch bot_description spawn.launch.py

To spawn the robot in an empty world and to control it with teleop_keyboard node run (A new terminal will open for teleop keyboard node): ros2 launch bot_desciption control.launch.py

To launch the robot in the custom world and contrl it (In this the robot will spawn in the customworld along with the rviz, and a new terminal will open for teleop keyboard node): ros2 launch bot_world bot_world.launch.py 

To read the laser scan data you can either run  (Note that after this make sure you've run a command to spawn the robot in gazebo in a different terminal): ros2 run bot_control reading_laser

For reading the laser scan data, spawning the robot in gazebo and rviz, also running the teleop keyboard all together: ros2 launch bot_control bot_control.launch.py
