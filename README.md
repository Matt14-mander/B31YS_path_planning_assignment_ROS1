# B31YS Assignment #

This is the code for the assignment on path planning. It is based on the ROS packages for ROSbot 2.0 and ROSbot 2.0 Pro found here [rosbot-packages](https://github.com/husarion/rosbot_ros/tree/noetic).
It presents how to run an autonomous mapping and navigation demo with ROSbot and Navigation2 stack.

# Quick start (simulation) #

## Installation ##

We assume that you are working on Ubuntu 20.04 and already have installed ROS Kinetic. If not, follow the [ROS install guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Prepare the repository:
```bash
cd ~
mkdir catkin_ws
mkdir catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```

Above commands should execute without any warnings or errors.

Clone this repository to your workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/IgnacioCarlucho/B31YS_path_planning_assignment_ROS1
```

Install dependencies:

```bash
cd ~/catkin_make
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd ~/ros_workspace
catkin_make
```

From this moment you can use rosbot simulations. Please remember that each time, when you open new terminal window, you will need to load system variables:

```bash
source ~/catkin_make/devel/setup.bash
```

If not you should add them to your bashrc. 




