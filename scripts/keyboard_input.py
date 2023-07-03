#!/usr/bin/python3

import readchar
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Twist
from std_msgs.msg import Bool
from relaxed_ik_ros2.msg import EEPoseGoals, EEVelGoals
import transformations as T
from robot import Robot
from pynput import keyboard
from ament_index_python.packages import get_package_share_directory

path_to_src = get_package_share_directory('relaxed_ik_ros2') + '/relaxed_ik_core'

class KeyboardInput(Node):
    def __init__(self):
        super().__init__('keyboard_input')

        deault_setting_file_path = path_to_src + '/configs/settings.yaml'
        
        setting_file_path = deault_setting_file_path
        try:
            setting_file_path = self.get_parameter('setting_file_path')
        except:
            pass

        self.robot = Robot(setting_file_path)

        self.ee_vel_goals_pub = self.create_publisher(EEVelGoals, 'relaxed_ik/ee_vel_goals', 5)

        self.pos_stride = 0.005
        self.rot_stride = 0.010

        self.seq = 1
        
        self.linear = [0.0,0.0,0.0]
        self.angular = [0.0,0.0,0.0]

        keyboard_listener = keyboard.Listener(
            on_press = self.on_press,
            on_release = self.on_release)

        self.create_timer(0.033, self.timer_callback)

        keyboard_listener.start()

    def on_press(self, key):
        self.linear = [0.0,0.0,0.0]
        self.angular = [0.0,0.0,0.0]

        if key.char == 'w':
            self.linear[0] += self.pos_stride
        elif key.char == 'x':
            self.linear[0] -= self.pos_stride
        elif key.char == 'a':
            self.linear[1] += self.pos_stride
        elif key.char == 'd':
            self.linear[1] -= self.pos_stride
        elif key.char == 'q':
            self.linear[2] += self.pos_stride
        elif key.char == 'z':
            self.linear[2] -= self.pos_stride
        elif key.char == '1':
            self.angular[0] += self.rot_stride
        elif key.char == '2':
            self.angular[0] -= self.rot_stride
        elif key.char == '3':
            self.angular[1] += self.rot_stride
        elif key.char == '4':
            self.angular[1] -= self.rot_stride
        elif key.char == '5':
            self.angular[2] += self.rot_stride
        elif key.char == '6':
            self.angular[2] -= self.rot_stride
        elif key.char == 'c':
            rclpy.shutdown()
        
        print("   Linear Vel: {}, Angular Vel: {}".format(self.linear, self.angular))

    def on_release(self, key):
        self.linear = [0.0,0.0,0.0]
        self.angular = [0.0,0.0,0.0]

    def timer_callback(self):
        msg = EEVelGoals()

        for i in range(self.robot.num_chain):
            twist = Twist()
            twist.linear.x = self.linear[0]
            twist.linear.y = self.linear[1]
            twist.linear.z = self.linear[2]

            twist.angular.x = self.angular[0]
            twist.angular.y = self.angular[1]
            twist.angular.z = self.angular[2]

            tolerance = Twist()
            tolerance.linear.x = 0.0
            tolerance.linear.y = 0.0
            tolerance.linear.z = 0.0
            tolerance.angular.x = 0.0
            tolerance.angular.y = 0.0
            tolerance.angular.z = 0.0

            msg.ee_vels.append(twist)
            msg.tolerances.append(tolerance)

        self.seq += 1
        self.ee_vel_goals_pub.publish(msg)
        
        
if __name__ == '__main__':
    rclpy.init()
    keyboard_input = KeyboardInput()
    rclpy.spin(keyboard_input)
    keyboard_input.destroy_node()
    rclpy.shutdown()
