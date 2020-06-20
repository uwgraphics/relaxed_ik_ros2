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
# may raise PackageNotFoundError
import ctypes

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]


package_share_directory = get_package_share_directory('relaxed_ik_ros2')
os.chdir(package_share_directory + '/relaxed_ik_core')

lib = ctypes.cdll.LoadLibrary(package_share_directory + '/relaxed_ik_core/target/debug/librelaxed_ik_ros2.so')
lib.rust_run.restype = Opt

# from cffi import FFI
# ffi = FFI()
# ffi.cdef("""
#     double* run(double **pos_goals, double **quat_goals);
# """)
# C = ffi.dlopen(package_share_directory + '/relaxed_ik_core/target/debug/librelaxed_ik_ros2.so')

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

def main(args=None):
    global eepg

    print("\nSolver initialized!\n", flush=True)

    rclpy.init()
    node = rclpy.create_node('relaxed_ik')
    node.create_subscription(EEPoseGoals, '/relaxed_ik/ee_pose_goals', eePoseGoals_cb, 3)

    angles_pub = node.create_publisher(JointAngles, '/relaxed_ik/joint_angle_solutions', 3)

    rclpy.spin_once(node)

    print("\nHeard!\n", flush=True)

    rate = node.create_rate(3000.0)

    while rclpy.ok():
        pose_goals = eepg.ee_poses
        header = eepg.header
        pos_arr = (ctypes.c_double * (3 * len(pose_goals)))()
        quat_arr = (ctypes.c_double * (4 * len(pose_goals)))()

        # print(type(pos_arr), flush=True)

        for i in range(len(pose_goals)):
            p = pose_goals[i]
            pos_arr[3*i] = p.position.x
            pos_arr[3*i+1] = p.position.y
            pos_arr[3*i+2] = p.position.z

            quat_arr[3*i] = p.orientation.w
            quat_arr[3*i+1] = p.orientation.x
            quat_arr[3*i+2] = p.orientation.y
            quat_arr[3*i+3] = p.orientation.z

        xopt = lib.rust_run(pos_arr, len(pos_arr), quat_arr, len(quat_arr))

        # print(xopt.data[0], flush=True)

        ja = JointAngles()
        ja.header = header
        for i in range(xopt.length):
            ja.angles.data.append(xopt.data[i])

        angles_pub.publish(ja)

        rate.sleep()

    rclpy.shutdown()


if __name__ == '__main__':
    main()