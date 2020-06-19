# relaxed_ik_ros2

This is Relaxed IK wrapped up in ROS2.

## Run
1. Configure the name of the pre-computed robot you would like to run with  (available options are ur5, yumi, panda and iiwa7) in relaxed_ik_core/config/loaded_robot.
2. Run the following command:
```
ros2 launch relaxed_ik_ros2 relaxed_ik_rust.launch
```