
## Overview
This tutorial explains at a basic level how to use ROS2 and PX4 in order to control a simulated UAV's velocity with keyboard controls. The goal is to create a simple example that a complete beginner can follow and understand, even with no ROS2 or PX4 experience.

### Prerequisites
* ROS2 Humble
* PX4 Autopilot
* Micro XRCE-DDS Agent
* px4_msgs
* Ubuntu 22.04
* Python 3.10


## Setup Steps

### Install PX4 Autopilot
To [Install PX4](https://docs.px4.io/main/en/dev_setup/building_px4.html#download-the-px4-source-code) run this code 
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```


### ROS2 Humble
To install ROS2 Humble follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Install Python dependencies as mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2) with this code

```
pip3 install --user -U empy pyros-genmsg setuptools
```

### Build Micro DDS
As mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#setup-micro-xrce-dds-agent-client) run this code in order to build MicroDDS on your machine

```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Clone Repo
This git repo is structured like a ROS2 workspace. Once you clone it, and build it, you should be able to run the code in the next section.

Run this code to clone the repo

```
cd ~
git clone https://github.com/ARK-Electronics/ws_ROS2_PX4_Simulation.git
```

### Building the Workspace
The two packages in this repo are px4_msgs and px4_offboard. px4_msgs is a ROS2 package that contains the ROS2 message definitions for PX4. px4_offboard is a ROS2 package that contains the code for the offboard control node that we will implement.

To build these two packages run this code

```
cd ~/ws_ROS2_PX4_Simulation
colcon build
```

After this runs, we should never need to build px4_msgs again. However, we will need to build px4_offboard every time we make changes to the code. To do this, and save time, we can run
```
colcon build --packages-select px4_offboard
```

If you tried to run our code now, it would not work. This is because we need to install the px4_msgs package into our ROS2 installation. To do this, run this code

```
cd ~/ws_ROS2_PX4_Simulation
source install/setup.bash
```

We will run this every time we build. It will also need to be run in every terminal that we want to run ROS2 commands in.


### Running the Code
This example has been designed to run from one launch file that will start all the necessary nodes. The launch file will run a python script that uses gnome terminal to open a new terminal window for MicroDDS and Gazebo.

Run this code to start the example

```
ros2 launch px4_offboard offboard_velocity_control.launch.py
```

This will run numerous things. In no particular order, it will run:

* processes.py in a new window
   * MicroDDS in a new terminal window
   * Gazebo will open in a second tab in the same terminal window
      * Gazebo GUI will open in it's own window
* control.py in a new window
   * Sends ROS2 Teleop commands to the /offboard_velocity_cmd topic based on keyboard input
* RVIZ will open in a new window
* velocity_control.py runs as it's own node, and is the main node of this example

Once everything is running, the UAV should arm and be ready for takeoff. You should be able to focus into the control.py terminal window and use the keyboard to control the UAV. The controls mimic Mode 2 RC Transmitter controls with WASD being the left joystick and the arrow keys being the right joystick. The controls are as follows:
* W: Up
* S: Down
* A: Yaw Left
* D: Yaw Right
* Up Arrow: Pitch Forward
* Down Arrow: Pitch Backward
* Left Arrow: Roll Left
* Right Arrow: Roll Right

Using the controls, click *W* to send a vertical veloctiy command and take off. Once in the air you can control it as you see fit.

## Closing Simulation *IMPORTANT*
When closing the simulation, it is very tempting to just close the terminal windows. However, this will leave Gazebo running in the background, potentially causing issues when you run Gazebo in the future. To correctly end the Gazebo simulation, go to it's terminal window and click *Ctrl+C*. This will close Gazebo and all of it's child processes. Then, you can close the other terminal windows.
 

 ## Explanation of processes.py
 This code runs each set of bash commands in a new tab of a gnome terminal window. It assumes that your PX4 installation is accessible from your root directory, and it is using the gz_x500 simulation. There is no current implementation to change these commands when running the launch file, however you can modify the command string within processes.py to change these values to what you need.

 If line 17 of processes.py were uncommented
```
17     # "cd ~/QGroundControl && ./QGroundControl.AppImage"
```
then QGroundControl would run in a new tab of the terminal window and the QGroundControl GUI would then open up. This is commented out by default because it is not necessary for the simulation to run, but it is useful for debugging, and is a simple example showing how to add another command to the launch file.



