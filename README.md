# Welcome to GalaxSea's code base!

Here's a tutorial on how to get this repo set up in your docker container


# Installation

CD into your workspace source (should be called roboboat_ws/src) and clone this repo
```shell
cd /root/roboboat_ws/src
git clone https://github.com/RoboSub-UTD/galaxsea
```

# Setup
**important:** don't run this command in your src directory, it must be in the workspace directory (roboboat_ws)
1. In your workspace directory (roboboat_ws), run 
```shell
colcon build --merge-install
```
2. Source the installation
```shell
. install/setup.bash
```

# Simulator + Controller

1. Launch the gazebo simulation using
```shell
ros2 launch vrx_gz competition.launch.py
```
After it launches, you should see a Gazebo GUI with a boat on the water. If you don't, try running 
```shell
xhost +
```
in a new terminal in your host computer. If it still fails, check your run script for allowing displays.

2. Run the keyboard boat controller using
```shell
ros2 run roboboat2025_ros2 boat_controller
```
Using waxd you can move the boat around the lake

# Setting up Custom Worlds

1. Add the path to the custom models and worlds to $GZ_SIM_RESOURCE_PATH

```shell
export GZ_SIM_RESOURCE_PATH="/root/roboboat_ws/src/galaxsea/custom_simulations/worlds:/root/roboboat_ws/src/galaxsea/custom_simulations/models:$GZ_SIM_RESOURCE_PATH"
```

Make sure to put your path in case it is different

2. Launch the new world using 
```shell
ros2 launch vrx_gz competition.launch.py world:=task_one
```
change task_one to whatever the world is called