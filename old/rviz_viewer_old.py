#! /usr/bin/env python3
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 5/8/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''

# from start_here import urdf_file_name, fixed_frame, joint_ordering, starting_config, joint_state_define
import rclpy
import yaml
import launch
import launch_ros.actions
import tf2_ros
import transformations as T
import os
import sys
import time
# from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from relaxed_ik_ros2.msg import JointAngles
import geometry_msgs.msg
from ament_index_python.packages import get_package_share_directory

ja_solution = ''
def ja_solution_cb(data):
    global ja_solution
    ja_solution = []
    for a in data.angles.data:
        ja_solution.append(a)

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node("rviz_viewer")
    # rospy.init_node('rviz_viewer')

    # may raise PackageNotFoundError
    package_share_directory = get_package_share_directory('relaxed_ik_ros2')
    # path_to_src = os.path.dirname(__file__)

    info_file_name = open(package_share_directory + '/relaxed_ik_core/config/loaded_robot', 'r').read()
    info_file_path = package_share_directory + '/relaxed_ik_core/config/info_files/' + info_file_name
    info_file = open(info_file_path, 'r')

    # print(info_file_path)

    y = yaml.load(info_file)
    # if not y == None:
    urdf_file_name = y['urdf_file_name']
    fixed_frame = y['fixed_frame']
    joint_ordering = y['joint_ordering']
    starting_config = y['starting_config']
    joint_state_define_file_name = y['joint_state_define_func_file']
    joint_state_define_file = open(package_share_directory + '/relaxed_ik_core/config/joint_state_define_functions/' + joint_state_define_file_name, 'r')
    joint_state_define = joint_state_define_file.read()
    exec(joint_state_define)

    urdf_path = package_share_directory + '/relaxed_ik_core/config/urdfs/' + urdf_file_name
    urdf_file = open(urdf_path, 'r')
    urdf_string = urdf_file.read()

    # node.declare_parameter('robot_description', urdf_string)
    js_pub = node.create_publisher(JointState,'joint_states',5)
    node.create_subscription(JointAngles,'/relaxed_ik/joint_angle_solutions',ja_solution_cb,5)
    tf_pub = tf2_ros.StaticTransformBroadcaster(node)

    time.sleep(0.5)

    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2',
            name='rviz2',
            node_executable='rviz2',
            # arguments=[package_share_directory + "/relaxed_ik_core/rviz/joint_viewer.rviz"]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            node_executable='robot_state_publisher',
            arguments=[urdf_path],
            parameters=[{'publish_frequency': '50.0'}],
        )
    ])
    print(launch.LaunchIntrospector().format_launch_description(ld))
    ls = launch.LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(ld)
    ls.run()

    urdf_pub = node.create_publisher(String, '/robot_description', 1)
    urdf_msg = String()
    urdf_msg.data = urdf_string
    urdf_pub.publish(urdf_msg)

    # uuid = launch_ros.rlutil.get_or_generate_uuid(None, False)
    # launch.configure_logging(uuid)
    # launch_path = package_share_directory + '/launch/joint_state_pub_nojsp.launch'
    # launch.parent.ROSLaunchParent(uuid, [launch_path]).start()

    prev_state = []

    # rate = rospy.Rate(200.0)
    prev_sol = starting_config
    while rclpy.ok():
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = fixed_frame
        t.child_frame_id = "common_world"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = T.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        tf_pub.sendTransform(t)

        if len(ja_solution) == 0:
            xopt = starting_config
        else:
            xopt = ja_solution
            if not len(xopt) == len(starting_config):
                xopt = prev_sol
            else:
                prev_sol = xopt

        js = joint_state_define(xopt)
        if js == None:
            js = JointState()
            js.name = joint_ordering
            for x in xopt:
                js.position.append(x)
        now = node.get_clock().now().to_msg()
        js.header.stamp.sec = now.sec
        js.header.stamp.nanosec = now.nanosec
        js_pub.publish(js)

        time.sleep(0.2)
	
    node.destroy_node()
    rclpy.shutdown()
    # time.sleep()