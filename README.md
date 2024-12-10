# Relaxed IK ROS 2

This is Relaxed IK wrapped up in ROS2. **The current version of this repository is made for ROS 2 Iron / Ubuntu 22.04**. It may also work on other ROS 2 versions.

You can find an introduction and the citation information in the README of [RelaxedIK Core](https://github.com/uwgraphics/relaxed_ik_core/tree/ranged-ik) which is a submodule of this repo. It is recommended to look at [RelaxedIK Core](https://github.com/uwgraphics/relaxed_ik_core) before working with this wrapper.

## The RelaxedIK Family
[RangedIK](https://github.com/uwgraphics/relaxed_ik_core/tree/ranged-ik) extends [RelaxedIK](https://github.com/uwgraphics/relaxed_ik_core/tree/relaxed-ik) by leveraging the flexibility afforded by tolerances. Also, RangedIK is recently maintained and works with more recent rust versions.

[CollisionIK](https://github.com/uwgraphics/relaxed_ik_core/tree/collision-ik) extends [RelaxedIK](https://github.com/uwgraphics/relaxed_ik_core/tree/relaxed-ik) by avoiding collisions with static or dynamic obstacles in the environment.

We provide a series of wrappers for our tools to be used in various platform or software. 

||**ROS1**|**ROS2**|**WebAssembly**|**Coppeliasim**|**Mujoco**|  
|:------|:-----|:-----|:-----|:-----|:-----| 
|**RangedIK**|[link](https://github.com/uwgraphics/relaxed_ik_ros1/tree/ranged-ik)|[link](https://github.com/uwgraphics/relaxed_ik_ros2)|[link](https://github.com/yepw/relaxed-ik-web-demo/)|x|x|  
|**CollisionIK**|[link](https://github.com/uwgraphics/relaxed_ik_ros1/)|x|x|x|x|  
|**RelaxedIK**|[link](https://github.com/uwgraphics/relaxed_ik_ros1/)|x|x|[link](https://github.com/uwgraphics/relaxed_ik_coppeliasim)|[link](https://github.com/uwgraphics/relaxed_ik_mujoco)|  


## Prerequisites
- You will need to [install ROS 2](https://docs.ros.org/en/iron/Installation.html)
    - If you already have [Conda](https://docs.conda.io/en/latest/) for Python installed on your system, you may run into errors with your python interpreter. A partial fix to this is to make sure you always `conda deactivate` before running any ROS commands, and to set `export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3` to make sure colcon builds with your system Python instead of Conda.
- You will need to [install Rust](https://www.rust-lang.org/tools/install) in order to build relaxed_ik_core (currently a necessary step to run RelaxedIK)
- You will need a [ros 2 workspace](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) on your system

## Installation
1. Clone this repo to `<your ros2 workspace>/src`

2. Clone git submodules
```bash
cd relaxed_ik_ros2
git submodule init
git submodule update
``` 

3. Compile relaxed_ik_core
```bash
cd relaxed_ik_core
cargo build
```

4. Clone robot descriptions under `your_workspace/src`. 
    * UR5 (Foxy): https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy (you can only keep the ur_description package) 

4. Install the UR Description package for ROS 2. For ROS 2 Galactic and later (incl. Humble, Iron), this package has [its own github repository](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description). For ROS 2 Foxy, it is included in the [UR Driver Repository](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy).


5. Build the workspace and source it:
```bash
colcon build --symlink-install
. install/setup.bash
```

## Verify Your Installation
Run the demo script below to verify your RelaxedIK installation. This script visualizes the output in Rviz and does not require a real robot.

1. Run relaxed_ik and a rviz viewer:
```bash
ros2 launch relaxed_ik_ros2 demo.launch.py
```

2.  To move the robot using keyboard, open a new terminal
```bash
ros2 run relaxed_ik_ros2 keyboard_input.py 
```

And then use the folloing commands to move the robot.
```bash
c - kill the controller controller script
w - move end effector along +X
x - move end effector along -X
a - move end effector along +Y
d - move end effector along -Y
q - move end effector along +Z
z - move end effector along -Z
1 - rotate end effector around +X
2 - rotate end effector around -X
3 - rotate end effector around +Y
4 - rotate end effector around -Y
5 - rotate end effector around +Z
6 - rotate end effector around -Z
```

## TODO
[] Visualize end-effector goal poses in Rviz 

[] Add a service for relaxed-ik (currently you can only use topic)
