# relaxed_ik_ros2

This is Relaxed IK wrapped up in ROS2. **Currently it's only tested on ROS2 Foxy**

## Prerequisites
- You will need ROS 2 installed on your system
- You will need to [install Rust](https://www.rust-lang.org/tools/install) if you haven't already in order to build relaxed_ik_core.

## Installation
1. Clone this repo to `your_workspace/src`

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


5. Build the workspace and source the package:
```bash
colcon build --symlink-install
. install/setup.bash
```
## Running
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
w - move chain 1 along +X
x - move chain 1 along -X
a - move chain 1 along +Y
d - move chain 1 along -Y
q - move chain 1 along +Z
z - move chain 1 along -Z
1 - rotate chain 1 around +X
2 - rotate chain 1 around -X
3 - rotate chain 1 around +Y
4 - rotate chain 1 around -Y
5 - rotate chain 1 around +Z
6 rotate chain 1 around -Z
```

## TODO
[] Test on a newer version of ROS2, e.g., humble.

[] Visualize end-effector goal poses in Rviz 

[] Add a service for relaxed-ik (currently you can only use topic)
