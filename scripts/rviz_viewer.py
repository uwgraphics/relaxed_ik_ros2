#! /usr/bin/env python3

'''
author: Danny Rakita, Haochen Shi, Yeping Wang
email: rakita@cs.wisc.edu, hshi74@wisc.edu, yeping@cs.wisc.edu
last update: 01/24/23
'''

import numpy
import os
import launch
import launch_ros.actions
import rclpy
import transformations as T
import yaml
from rclpy.node import Node
import time

from robot import Robot
# from interactive_markers.interactive_marker_server import *
from sensor_msgs.msg import JointState, PointCloud2, PointField
from visualization_msgs.msg import *
import subprocess
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Point
from relaxed_ik_ros2.msg import EEPoseGoals, EEVelGoals
from ament_index_python.packages import get_package_share_directory

path_to_src = get_package_share_directory('relaxed_ik_ros2')

class RvizViewer(Node):
    def __init__(self):
        super().__init__('rviz_viewer')

        self.declare_parameter('setting_file_path', rclpy.Parameter.Type.STRING)

        deault_setting_file_path = path_to_src + '/relaxed_ik_core/configs/settings.yaml'

        setting_file_path = deault_setting_file_path
        try:
            setting_file_path = self.get_parameter('setting_file_path').value
            self.get_logger().info("Using setting file " + setting_file_path)
        except:
            self.get_logger().info("No setting file path is given, using default setting file" + setting_file_path)

        # Load the infomation
        setting_file = open(setting_file_path, 'r')
        settings = yaml.load(setting_file, Loader=yaml.FullLoader)

        self.robot = Robot(setting_file_path)

        self.js_pub = self.create_publisher(JointState,'joint_states', 5)
        self.js_msg = JointState()
        self.js_msg.name = self.robot.all_joint_names
        self.js_msg.position = [0.0] * len(self.robot.all_joint_names)

        # Markers to visualize goal poses
        # self.server = InteractiveMarkerServer("simple_marker")

        if 'starting_config' not in settings:
            settings['starting_config'] = [0.0] * len(self.robot.articulated_joint_names)
        else:
            self.starting_config = settings['starting_config']

        self.ee_poses =  self.robot.fk(self.starting_config)
        
        # for i in range(self.robot.num_chain):
        #     pose_goal_marker = make_marker('arm_'+str(i), settings['base_links'][i],
        #          'widget', [0.1,0.1,0.1], self.ee_poses[i], False)
        #     self.server.insert(pose_goal_marker)

        # wait for robot state publisher to start
        time.sleep(2.0)
        
        # move the robot in rviz to initial position
        for i in range(len(self.robot.articulated_joint_names)):
            self.js_msg.position[self.robot.all_joint_names.index(self.robot.articulated_joint_names[i])] = \
                                                            self.starting_config[i]
        self.js_msg.header.stamp = self.get_clock().now().to_msg()
        self.js_pub.publish(self.js_msg)

        self.create_subscription(JointState, '/relaxed_ik/joint_angle_solutions', self.ja_solution_cb, 1)
        # self.create_subscription(EEPoseGoals, '/relaxed_ik/vis_ee_poses', self.ee_pose_goal_cb, 1)
        # self.create_subscription(EEPoseGoals, '/relaxed_ik/ee_pose_goals', self.ee_pose_goal_cb, 1)
        # self.create_subscription(EEVelGoals, '/relaxed_ik/ee_vel_goals', self.ee_vel_goal_cb, 1)

    def ja_solution_cb(self, msg):
        self.js_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(msg.name)):
            self.js_msg.position[self.robot.all_joint_names.index(msg.name[i])] = msg.position[i]
        self.js_pub.publish(self.js_msg)

    def ee_pose_goal_cb(self, msg):
        assert len(msg.ee_poses) == self.robot.num_chain
        for i in range(self.robot.num_chain):
            self.ee_poses[i] = msg.ee_poses[i]
        # self.update_marker()

    def ee_vel_goal_cb(self, msg):
        assert len(msg.ee_vels) == self.robot.num_chain
        for i in range(self.robot.num_chain):
            self.ee_poses[i].position.x += msg.ee_vels[i].linear.x
            self.ee_poses[i].position.y += msg.ee_vels[i].linear.y
            self.ee_poses[i].position.z += msg.ee_vels[i].linear.z
            curr_q = [self.ee_poses[i].orientation.w, self.ee_poses[i].orientation.x, self.ee_poses[i].orientation.y, self.ee_poses[i].orientation.z]
            tmp_q = T.quaternion_from_scaledAxis([msg.ee_vels[i].angular.x, msg.ee_vels[i].angular.y, msg.ee_vels[i].angular.z])
            after_q = T.quaternion_multiply(tmp_q, curr_q)
            self.ee_poses[i].orientation.w = after_q[0]
            self.ee_poses[i].orientation.x = after_q[1]
            self.ee_poses[i].orientation.y = after_q[2]
            self.ee_poses[i].orientation.z = after_q[3]
        self.update_marker()

    def update_marker(self):
        for i in range(self.robot.num_chain):
            self.server.setPose('arm_'+str(i), self.ee_poses[i])
        self.server.applyChanges()

def make_marker(name, fixed_frame, shape, scale, pose, is_dynamic, 
                points=None, color=[0.0,0.5,0.5,1.0], marker_scale=0.3):                
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = fixed_frame
    int_marker.name = name
    int_marker.pose = pose

    int_marker.scale = marker_scale

    origin = Point()
    x_axis = Point()
    x_axis.x = scale[0]
    y_axis = Point()
    y_axis.y = scale[1]
    z_axis = Point()
    z_axis.z = scale[2]
    points = [[origin, x_axis], [origin, y_axis], [origin, z_axis]]
    colors = [[1.0, 0.0, 0.0, 0.6], [0.0, 1.0, 0.0, 0.6], [0.0, 0.0, 1.0, 0.6]]
    for i in range(len(colors)):
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0.03
        marker.color.r = colors[i][0]
        marker.color.g = colors[i][1]
        marker.color.b = colors[i][2]
        marker.color.a = colors[i][3]
        marker.points = points[i]

        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
  
    return int_marker

if __name__ == '__main__':
    rclpy.init()
    rviz_viewer = RvizViewer()
    rclpy.spin(rviz_viewer)
    rviz_viewer.destroy_node()
    rclpy.shutdown()
