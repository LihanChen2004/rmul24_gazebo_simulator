import sys
import threading
import numpy as np
import cv2
import rclpy
import copy
import time
import base64

from sensor_msgs.msg import Image
from rmoss_interfaces.msg import ChassisCmd
from rmoss_interfaces.msg import GimbalCmd
from rmoss_interfaces.msg import ShootCmd
from std_msgs.msg import Int32, String




def publish_chassis_cmd_msg(pub, x, y, w):
    # print('pub chassis')
    msg = ChassisCmd()
    msg.type = msg.FOLLOW_GIMBAL
    msg.twist.linear.x = x
    msg.twist.linear.y = y
    msg.twist.angular.z = w
    # print(msg)
    pub.publish(msg)


def publish_gimbal_cmd_msg(pub, pitch, yaw):
    # print('pub gimbal')
    msg = GimbalCmd()
    msg.pitch_type = msg.ABSOLUTE_ANGLE
    msg.yaw_type = msg.ABSOLUTE_ANGLE
    msg.position.yaw = yaw
    msg.position.pitch = pitch
    # print(msg)
    pub.publish(msg)


def publish_shoot_cmd_msg(pub, num, vel):
    # print('pub shoot')
    msg = ShootCmd()
    msg.projectile_num = num
    msg.projectile_velocity = vel
    # print(msg)
    pub.publish(msg)

def publish_reset_cmd_msg(pub):
    msg = Int32()
    msg.data = 500
    pub.publish(msg)

