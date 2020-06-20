#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
# from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_ik_ros2.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float64
# from RelaxedIK.Utils.colors import bcolors
# from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from ament_index_python.packages import get_package_share_directory
from cffi import FFI

# may raise PackageNotFoundError
package_share_directory = get_package_share_directory('relaxed_ik_ros2')

ffi = FFI()
ffi.cdef("""
    double* run(double **pos_goals, double **quat_goals);
""")
C = ffi.dlopen(package_share_directory + '/relaxed_ik_core/target/debug/librelaxed_ik_node_ros2.so')

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

def main(args=None):
    global eepg

    print("\nSolver initialized!\n")

    rclpy.init()
    node = rclpy.create_node('relaxed_ik')
    node.create_subscription(EEPoseGoals, '/relaxed_ik/ee_pose_goals', eePoseGoals_cb, 3)

    angles_pub = node.create_publisher(JointAngles, '/relaxed_ik/joint_angle_solutions', 3)

    rclpy.spin_once(node)

    rate = node.create_rate(3000.0)

    while rclpy.ok():
        pos_goals = []
        quat_goals = []
        pose_goals = eepg.ee_poses
        header = eepg.header

        for p in pose_goals:
            pos_x = p.position.x
            pos_y = p.position.y
            pos_z = p.position.z

            quat_w = p.orientation.w
            quat_x = p.orientation.x
            quat_y = p.orientation.y
            quat_z = p.orientation.z

            pos_goals.append([pos_x, pos_y, pos_z])
            quat_goals.append([quat_w, quat_x, quat_y, quat_z])

        xopt = C.run(pos_goals, quat_goals)
        ja = JointAngles()
        ja.header = header
        for x in xopt:
            ja.angles.data.append(x)

        angles_pub.publish(ja)
        # print xopt

        rate.sleep()

    rclpy.shutdown()


if __name__ == '__main__':
    main()