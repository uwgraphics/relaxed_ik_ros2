#!/usr/bin/python3

import ctypes
import numpy as np
import os
import sys
import transformations as T
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# from relaxed_ik_ros2.srv import IKPose, IKPoseResponse
from relaxed_ik_ros2.msg import EEPoseGoals, EEVelGoals
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState 
from urdf_parser_py.urdf import URDF
from kdl_parser import kdl_tree_from_urdf_model
import PyKDL as kdl
from robot import Robot

path_to_src = get_package_share_directory('relaxed_ik_ros2') + '/relaxed_ik_core'
sys.path.insert(1, path_to_src + '/wrappers')
from python_wrapper import RelaxedIKRust

class RelaxedIK(Node):
    def __init__(self):
        super().__init__('relaxed_ik')
        self.declare_parameter('setting_file_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('use_visualization', rclpy.Parameter.Type.BOOL)

        default_setting_file_path = path_to_src + '/configs/settings.yaml'

        setting_file_path = ""
        try: 
            setting_file_path = self.get_parameter('setting_file_path').value
            self.get_logger().info("Using setting file " + setting_file_path)
        except:
            pass

        if setting_file_path == "":
            self.get_logger().info(f"No setting file path is given, using the default setting file {default_setting_file_path}")
            setting_file_path = default_setting_file_path

        try: 
            self.use_visualization = self.get_parameter('use_visualization').value
        except:
            self.use_visualization = False

        os.chdir(path_to_src )

        # Load the infomation
        
        print("setting_file_path: ", setting_file_path)
        setting_file = open(setting_file_path, 'r')
        settings = yaml.load(setting_file, Loader=yaml.FullLoader)
       
        urdf_file = open(path_to_src + '/configs/urdfs/' + settings["urdf"], 'r')
        urdf_string = urdf_file.read()
        self.declare_parameter('robot_description', urdf_string)

        self.relaxed_ik = RelaxedIKRust(setting_file_path)

        # Services
        # self.ik_pose_service = rospy.Service('relaxed_ik/solve_pose', IKPose, self.handle_ik_pose)

        # Publishers
        self.angles_pub = self.create_publisher(JointState, 'relaxed_ik/joint_angle_solutions', 1)
        if self.use_visualization:
            self.vis_ee_pub = self.create_publisher(EEPoseGoals, 'relaxed_ik/vis_ee_poses', 1)

        self.robot = Robot(setting_file_path)

        self.js_msg = JointState()
        self.js_msg.name = self.robot.articulated_joint_names
        self.js_msg.position = []

        if 'starting_config' not in settings:
            settings['starting_config'] = [0.0] * len(self.js_msg.name)
        else:
            assert len(settings['starting_config']) == len(self.js_msg.name), \
                    "Starting config length does not match the number of joints"
            for i in range(len(self.js_msg.name)):
                self.js_msg.position.append( settings['starting_config'][i] )
        
        # Subscribers
        self.create_subscription(EEPoseGoals, '/relaxed_ik/ee_pose_goals', self.pose_goals_cb, 1)
        self.create_subscription(EEVelGoals, '/relaxed_ik/ee_vel_goals', self.pose_vels_cb, 1)
        self.create_subscription(JointState, '/relaxed_ik/reset', self.reset_cb, 1)

        print("\nSolver RelaxedIK initialized!\n")

    def get_ee_pose(self):
        ee_poses = self.relaxed_ik.get_ee_positions()
        ee_poses = np.array(ee_poses)
        ee_poses = ee_poses.reshape((len(ee_poses)//6, 6))
        ee_poses = ee_poses.tolist()
        return ee_poses

    def handle_ik_pose(self, req):
        positions = []
        orientations = []
        tolerances = []
        for i in range(len(req.ee_poses)):
            positions.append(req.ee_poses[i].position.x)
            positions.append(req.ee_poses[i].position.y)
            positions.append(req.ee_poses[i].position.z)
            orientations.append(req.ee_poses[i].orientation.x)
            orientations.append(req.ee_poses[i].orientation.y)
            orientations.append(req.ee_poses[i].orientation.z)
            orientations.append(req.ee_poses[i].orientation.w)
            if i < len(req.tolerances):
                tolerances.append(req.tolerances[i].linear.x)
                tolerances.append(req.tolerances[i].linear.y)
                tolerances.append(req.tolerances[i].linear.z)
                tolerances.append(req.tolerances[i].angular.x)
                tolerances.append(req.tolerances[i].angular.y)
                tolerances.append(req.tolerances[i].angular.z)
            else:
                for j in range(6):
                    tolerances.append(0.0)

        if self.use_visualization:
            vis_msg = EEPoseGoals()
            vis_msg.ee_poses = req.ee_poses
            vis_msg.tolerances = req.tolerances
            self.vis_ee_pub.publish(vis_msg)

        ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        self.js_msg.header.stamp = self.get_clock().now().to_msg()
        self.js_msg.position = ik_solution
        self.angles_pub.publish(self.js_msg)
        res = IKPoseResponse()
        res.joint_state = ik_solution

        return res

    def reset_cb(self, msg):
        n = len(msg.positions)
        x = (ctypes.c_double * n)()
        for i in range(n):
            x[i] = msg.positions[i]
        self.relaxed_ik.reset(x, n)

    def pose_goals_cb(self, msg):
        positions = []
        orientations = []
        tolerances = []
        for i in range(len(msg.ee_poses)):
            positions.append(msg.ee_poses[i].position.x)
            positions.append(msg.ee_poses[i].position.y)
            positions.append(msg.ee_poses[i].position.z)
            orientations.append(msg.ee_poses[i].orientation.x)
            orientations.append(msg.ee_poses[i].orientation.y)
            orientations.append(msg.ee_poses[i].orientation.z)
            orientations.append(msg.ee_poses[i].orientation.w)
            if i < len(msg.tolerances):
                tolerances.append(msg.tolerances[i].linear.x)
                tolerances.append(msg.tolerances[i].linear.y)
                tolerances.append(msg.tolerances[i].linear.z)
                tolerances.append(msg.tolerances[i].angular.x)
                tolerances.append(msg.tolerances[i].angular.y)
                tolerances.append(msg.tolerances[i].angular.z)
            else:
                for j in range(6):
                    tolerances.append(0.0)

        ik_solution = self.relaxed_ik.solve_position(positions, orientations, tolerances)

        # Publish the joint angle solution
        self.js_msg.header.stamp = self.get_clock().now().to_msg()
        self.js_msg.position = ik_solution
        self.angles_pub.publish(self.js_msg)

    def pose_vels_cb(self, msg):
        linear_vels = []
        angular_vels = []
        tolerances = []
        for i in range(len(msg.ee_vels)):
            linear_vels.append(msg.ee_vels[i].linear.x)
            linear_vels.append(msg.ee_vels[i].linear.y)
            linear_vels.append(msg.ee_vels[i].linear.z)
            angular_vels.append(msg.ee_vels[i].angular.x)
            angular_vels.append(msg.ee_vels[i].angular.y)
            angular_vels.append(msg.ee_vels[i].angular.z)
            if i < len(msg.tolerances):
                tolerances.append(msg.tolerances[i].linear.x)
                tolerances.append(msg.tolerances[i].linear.y)
                tolerances.append(msg.tolerances[i].linear.z)
                tolerances.append(msg.tolerances[i].angular.x)
                tolerances.append(msg.tolerances[i].angular.y)
                tolerances.append(msg.tolerances[i].angular.z)
            else:
                for j in range(6):
                    tolerances.append(0.0)

        ik_solution = self.relaxed_ik.solve_velocity(linear_vels, angular_vels, tolerances)

        assert len(ik_solution) == len(self.robot.articulated_joint_names)

        # Publish the joint angle solution
        self.js_msg.header.stamp = self.get_clock().now().to_msg()
        self.js_msg.position = ik_solution
        self.angles_pub.publish(self.js_msg)

if __name__ == '__main__':
    rclpy.init()
    relaxed_ik = RelaxedIK()
    rclpy.spin(relaxed_ik)
    relaxed_ik.destroy_node()
    rclpy.shutdown()


