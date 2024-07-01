# ==========================================
#   packages for ros
# ==========================================
import rclpy
import sys
from sensor_msgs.msg import Image
import cv2
import base64
import time
import numpy as np
from rmoss_interfaces.msg import ChassisCmd
from rmoss_interfaces.msg import GimbalCmd
from rmoss_interfaces.msg import ShootCmd
from std_msgs.msg import Int32, String
# ==========================================
#    ros functions
# ==========================================
from ros_handler import *

# ==========================================
#    robot state
# ==========================================
gimbal_pitch = 0.0
gimbal_yaw = 0.0
speed = 1.0

# ==========================================
#    pubs
# ==========================================
node = None
robot_name = None
# chassis_cmd_pub = 0
# gimbal_cmd_pub = 0
# shoot_cmd_pub = 0

def  send_instruction(chassis_x, chassis_y, gimbal_pitch, gimbal_yaw):
    # publish_chassis_cmd_msg(chassis_cmd_pub, chassis_x, chassis_y, 0.0)
    publish_gimbal_cmd_msg(gimbal_cmd_pub, 0.0, gimbal_yaw)

def log_func(msg):
    print(msg.data)

if __name__ == '__main__':
    # global node, chassis_cmd_pub, gimbal_cmd_pub, shoot_cmd_pub

    #ROS2 Task
    rclpy.init()
    node=rclpy.create_node('player_web')
    #standard_robot_red1,standard_robot_blue1
    robot_name="standard_robot_red1"
    port=5000
    if len(sys.argv) == 3:
        robot_name = str(sys.argv[1])
    # print("Robot name:",robot_name, "Port:",port)
    # chassis_cmd_pub = node.create_publisher(ChassisCmd, '/%s/robot_base/chassis_cmd' % (robot_name), 10)
    # gimbal_cmd_pub = node.create_publisher(GimbalCmd, '/%s/robot_base/gimbal_cmd' % (robot_name), 10)
    # shoot_cmd_pub = node.create_publisher(ShootCmd, '/%s/robot_base/shoot_cmd' % (robot_name), 10)
    blue_hp_sub = node.create_subscription(Int32, '/referee_system/standard_robot_blue1/hp', log_func,
                                       10)
    red_hp_sub = node.create_subscription(Int32, '/referee_system/standard_robot_red1/hp', log_func,
                                       10)
    attack_info_sub = node.create_subscription(String, '/referee_system/attack_info', log_func,
                                       10)
    rclpy.spin(node)
    node.destroy_node()
    # gimbal_pitch = 0.0
    # gimbal_yaw = 0.0
    # while True:
    #     # send_instruction(0.0, 0.0, 0.0, gimbal_yaw)
    #     # # gimbal_pitch = gimbal_pitch + 0.1
    #     # gimbal_yaw = gimbal_yaw + 0.2
    #     # print(gimbal_yaw)
    #     time.sleep(0.5)