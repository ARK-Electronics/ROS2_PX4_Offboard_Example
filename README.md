
# ROS2_PX4_Offboard_Example

## Overview
This tutorial explains at a basic level how to use ROS2 and PX4 in order to control a simulated UAV's velocity with keyboard controls. The goal is to create a simple example that a complete beginner can follow and understand, even with no ROS2 or PX4 experience.

This repo is a derivative of Jaeyoung Lim's Offboard example
https://github.com/Jaeyoung-Lim/px4-offboard

I've taken his example and added some functionality. 

## YouTube Tutorial
We published a walkthrough tutorial on YouTube to demonstrate the example and to help beginners set up their enviornment. The video is helpful, but be sure to always defer to this Readme file for instructions. Some changes have been made since the video was posted, meaning that though it is helpful, it is not 100% accurate.

You can watch the video [here](https://www.youtube.com/watch?v=8gKIP0OqHdQ).

### Prerequisites
* ROS2 Humble
* PX4 Autopilot
* Micro XRCE-DDS Agent
* px4_msgs
* Ubuntu 22.04
* Python 3.10


## Setup Steps

### Install PX4 Autopilot
To [Install PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets) run this code 
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.15
```

Run this script in a bash shell to install everything

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

You will now need to restart your computer before continuing.


### Install ROS2 Humble
To install ROS2 Humble follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Install Dependencies

Install Python dependencies as mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2) with this code

```
pip3 install --user -U empy pyros-genmsg setuptools
```

I also found that without these packages installed Gazebo has issues loading

```
pip3 install kconfiglib
pip install --user jsonschema
pip install --user jinja2
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

### Setup Workspace
This git repo is intended to be a ROS2 package that is cloned into a ROS2 workspace.

We're going to create a workspace in our home directory, and then clone in this repo and also the px4_msgs repo. 

For more information on creating workspaces, see [here](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html)

Run this code to create a workspace in your home directory

```
mkdir -p ~/ros2_px4_offboard_example_ws/src
cd ~/ros2_px4_offboard_example_ws/src
```

*ros2_px4_offboard_example_ws* is just a name I chose for the workspace. You can name it whatever you want. But after we run *colcon build* you might have issues changing your workspace name so choose wisely.

We are now in the src directory of our workspace. This is where ROS2 packages go, and is where we will clone in our two repos.

### Clone in Packages
We first will need the px4_msgs package. Our ROS2 nodes will rely on the message definitions in this package in order to communicate with PX4. Read [here](https://docs.px4.io/main/en/ros/ros2_comm.html#overview:~:text=ROS%202%20applications,different%20PX4%20releases) for more information.

Be sure you're in the src directory of your workspace and then run this code to clone in the px4_msgs repo

```
git clone https://github.com/PX4/px4_msgs.git -b release/1.15
```

Once again be sure you are still in the src directory of your workspace. Run this code to clone in our example package

```
git clone https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git
```

Run this code to clone the repo



### Building the Workspace
The two packages in this workspace are px4_msgs and px4_offboard. px4_offboard is a ROS2 package that contains the code for the offboard control node that we will implement. It lives inside the ROS2_PX4_Offboard_Example directory.

Before we build these two packages, we need to source our ROS2 installation. Run this code to do that

```
source /opt/ros/humble/setup.bash
```

This will need to be run in every terminal that wants to run ROS2 commands. An easy way to get around this, is to add this command to your .bashrc file. This will run this command every time you open a new terminal window.

To build these two packages, you must be in workspace directory not in src, run this code to change directory from src to one step back i.e. root of your workspace and build the packages

```
cd ..
colcon build
```
As mentioned in Jaeyoung Lim's [example](https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/doc/ROS2_PX4_Offboard_Tutorial.md) you will get some warnings about setup.py but as long as there are no errors, you should be good to go.


After this runs, we should never need to build px4_msgs again. However, we will need to build px4_offboard every time we make changes to the code. To do this, and save time, we can run
```
colcon build --packages-select px4_offboard
```

If you tried to run our code now, it would not work. This is because we need to source our current workspace. This is always done after a build. To do this, be sure you are in the src directory, and then run this code

```
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

Once everything is running, you should be able to focus into the control.py terminal window, arm, and takeoff. The controls mimic Mode 2 RC Transmitter controls with WASD being the left joystick and the arrow keys being the right joystick. The controls are as follows:
* W: Up
* S: Down
* A: Yaw Left
* D: Yaw Right
* Up Arrow: Pitch Forward
* Down Arrow: Pitch Backward
* Left Arrow: Roll Left
* Right Arrow: Roll Right
* Space: Arm/Disarm

Pressing *Space* will arm the drone. Wait a moment and it will takeoff and switch into offboard mode. You can now control it using the above keys. If you land the drone, it will disarm and to takeoff again you will need to toggle the arm switch off and back on with the space bar. 

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

## Known Issues
If the vehicle does not arm when you press Enter, check to ensure the parameter NAV_DLL_ACT is set to 0. You may need to download QGroundControl and disable this parameter if you want to run this demo without needing QGC open.

## Questions
Join the ARK Electronics Discord [here](https://discord.gg/TDJzJxUMRX) for more help and to stay up to date on our projects.

