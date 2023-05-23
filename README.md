# ws_ROS2_PX4_Simulation
More in depth documentation and tutorial coming soon.

This repo is a derivative of Jaeyoung Lim's Offboard example
https://github.com/Jaeyoung-Lim/px4-offboard

I've taken his example and added some functionality. 

Everything runs from a single launch command.
  -MicroDDS
  -Gazebo
  -Teleop Terminal
  -ROS Node
  
Build the packages(should just be px4_msgs and px4_offboard)
From /src run source install/setup.bash
Run 
  ros2 launch px4_offboard offboard_velocity_control.launch.py

This should run everything needed. The drone should arm, takeoff, and then switch into offboard mode.
Focus into the teleop terminal and use WASD and the arrow keys to control the drone. 

*IMPORTANT*
When you close the simulation DO NOT 'X' out of the Gazebo simulation GUI. It will close to GUI but continue to run in the background.
You need to Ctrl+C in the terminal. I do this for everything just to be sure but Gazebo will error on the next run if you don't do this.

If you need to change specifics of what is launching in the launch script or if you need to launc QGC, visit processes.py or offboard_velocity_control.launch.py

Message Braden Wagstaff on the PX4 Discord for questoins or email me at braden@arkelectron.com
