# relaxed_ik_ros2

This is Relaxed IK wrapped up in ROS2.

## Run
1. Configure the name of the pre-computed robot you would like to run with  (available options are ur5, yumi, panda and iiwa7) in relaxed_ik_core/config/loaded_robot.

2. Build the workspace and source the package:
```
colcon build --symlink-install
. install/setup.bash
```

3. Run the following command:
```
ros2 launch relaxed_ik_ros2 relaxed_ik_rust.launch
```

4. Test by opening a new terminal and publishing a message to EE pose goals
```
. install/setup.bash
ros2 topic pub -1 /relaxed_ik/ee_pose_goals relaxed_ik_ros2/msg/EEPoseGoals "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, ee_poses: [{position: {x: 0.015, y: 0.015, z: 0.015}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}"
```

## Known issues
1. The ROS2 version cannot listen to ee_pose_goal messages after the first one.

2. I didn't figure out a way to implement the equivalence of executing a loop at a fixed rate in ROS2 with rclpy. rclpy.timer.Rate.sleep() does not work perfectly in this case.