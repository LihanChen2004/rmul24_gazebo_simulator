#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from rmoss_interfaces.msg import RefereeCmd,RobotStatus
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

def parse_attack_info(attack_str):
    # parse msg
    info = attack_str.split(',')
    if len(info) < 2:
        return None
    # shooter info
    shooter = info[0].split('/')
    if len(shooter) != 2:
        return None
    shooter_model_name = shooter[0]
    shooter_name = shooter[1]
    # target info
    target = info[1].split('/')
    if len(target) != 4:
        return None
    target_model_name = target[1]
    target_link_name = target[2]
    target_collision_name = target[3]
    # print("target info:",target)
    if target_collision_name != 'target_collision':
        return None
    return shooter_model_name, shooter_name, target_model_name, target_link_name

class StandardRobot:
    def __init__(self,node,robot_name):
        self.node = node
        self.initial_tf = None
        self.tf = None
        self.robot_name = robot_name
        status_topic = '/referee_system/' + robot_name + '/robot_status'
        self.status_pub = node.create_publisher(RobotStatus, status_topic, 10)
        enable_control_topic = '/referee_system/' + robot_name + '/enable_control'
        self.enable_control_pub = node.create_publisher(Bool, enable_control_topic, 10)
        enable_power_topic = '/referee_system/' + robot_name + '/enable_power'
        self.enable_power_pub = node.create_publisher(Bool, enable_power_topic, 10)
        self.reset_data()

    def reset_data(self):
        self.max_hp = self.node.get_parameter("max_hp").get_parameter_value().integer_value
        self.total_projectiles = self.node.get_parameter("initial_projectiles").get_parameter_value().integer_value
        self.remain_hp = self.max_hp
        self.used_projectiles = 0
        self.hit_projectiles = 0
        self.survive = True

    def enable_power(self, enable):
        msg = Bool()
        msg.data = enable
        self.enable_power_pub.publish(msg)

    def enable_control(self, enable):
        msg = Bool()
        msg.data = enable
        self.enable_control_pub.publish(msg)

    def update_hp(self, hp):
        self.remain_hp = self.remain_hp + hp
        if self.remain_hp < 0:
            self.remain_hp = 0
        if self.remain_hp > self.max_hp:
            self.remain_hp = self.max_hp

    def update_tf(self, tf):
        self.tf = tf
        if self.initial_tf is None:
            self.initial_tf = tf

    def supply_projectile(self,num):
        self.total_projectiles = self.total_projectiles + num

    def consume_projectile(self):
        self.used_projectiles = self.used_projectiles + 1

    def record_hit(self):
        self.hit_projectiles = self.hit_projectiles + 1

    def publish_status(self):
        msg = RobotStatus()
        msg.max_hp = self.max_hp
        msg.remain_hp = self.remain_hp
        msg.total_projectiles = self.total_projectiles
        msg.used_projectiles = self.used_projectiles
        msg.hit_projectiles = self.hit_projectiles
        if self.tf is not None:
            msg.gt_tf = self.tf
        self.status_pub.publish(msg)

class SimpleRefereeSystem():
    def __init__(self,node):
        self.node = node
        self.node.declare_parameter('max_hp', 500)
        self.node.declare_parameter('initial_projectiles', 100)
        self.attack_info_sub = node.create_subscription(
            String,
            '/referee_system/ign/attack_info',
            self.attack_info_callback,
            50)
        self.shoot_info_sub = node.create_subscription(
            String,
            '/referee_system/ign/shoot_info',
            self.shoot_info_callback,
            50)
        self.pose_info_sub = node.create_subscription(
            TFMessage,
            '/referee_system/ign/pose_info',
            self.pose_info_callback,
            50)
        self.set_pose_pub = node.create_publisher(
            TransformStamped,
            '/referee_system/ign/set_pose',
            10)
        self.referee_cmd_sub = node.create_subscription(
            RefereeCmd,
            '/referee_system/referee_cmd',
            self.referee_cmd_callback,
            1)
        self.robots = {}
        self.robots['red_standard_robot1'] = StandardRobot(node = node, robot_name = 'red_standard_robot1')
        self.robots['blue_standard_robot1'] = StandardRobot(node = node, robot_name = 'blue_standard_robot1')
        self.timer_cb = node.create_timer(0.5, self.timer_cb)
        self.attack_info_sub  # prevent unused variable warning
        self.timer_cb  # prevent unused variable warning
        self.game_over = False

    def attack_info_callback(self, msg: String):
        if self.game_over:
            return
        # parse msg
        data = parse_attack_info(msg.data)
        if data is None:
            return
        shooter_model_name = data[0]
        target_model_name = data[2]
        target_link_name = data[3]
        # process attack info
        if 'armor' in target_link_name:
            if target_model_name in self.robots.keys():
                self.robots[target_model_name].update_hp(-10)
        if shooter_model_name in self.robots.keys():
            self.robots[shooter_model_name].record_hit()

    def shoot_info_callback(self, msg: String):
        if self.game_over:
            return
        info = msg.data.split(',')
        if len(info) < 2:
            return
        shooter = info[0].split('/')
        if len(shooter) != 2:
            return
        shooter_model_name = shooter[0]
        # shooter_name = shooter[1]
        vel = float(info[1])
        if shooter_model_name not in self.robots.keys():
            return
        self.robots[shooter_model_name].consume_projectile()
        if vel > 30:
            self.robots[shooter_model_name].update_hp(-10)

    def pose_info_callback(self, msg: TFMessage):
        for obj_tf in msg.transforms:
            if obj_tf.child_frame_id in self.robots.keys():
                self.robots[obj_tf.child_frame_id].update_tf(obj_tf.transform)

    def timer_cb(self):
        if self.game_over:
            return
        # check survive
        for robot in self.robots.values():
            if robot.survive and robot.remain_hp == 0:
                # kill robot
                robot.enable_power(False)
                robot.survive = False
        # check game over
        red_survive = False
        blue_survive = False
        for robot_name, robot in self.robots.items():
            if 'red' in robot_name:
                red_survive = red_survive or robot.survive
            if 'blue' in robot_name:
                blue_survive = blue_survive or robot.survive
        if red_survive == False or blue_survive == False:
            self.game_over = True
        # publish robot status
        for robot in self.robots.values():
            robot.publish_status()

    def referee_cmd_callback(self, msg: RefereeCmd):
        if msg.cmd == msg.PREPARATION:
            self.game_over = True
            for robot in self.robots.values():
                robot.enable_power(False)
            for robot_name, robot in self.robots.items():
                assert robot.initial_tf is not None
                msg = TransformStamped()
                msg.child_frame_id = robot_name
                msg.transform = robot.initial_tf
                self.set_pose_pub.publish(msg)
                time.sleep(0.05)
            for robot in self.robots.values():
                robot.enable_power(True)
        elif msg.cmd == msg.SELF_CHECKING:
            self.game_over = False
            for robot in self.robots.values():
                robot.reset_data()
                robot.enable_power(True)
                robot.enable_control(False)
        elif msg.cmd == msg.START_GAME:
            self.game_over = False
            for robot in self.robots.values():
                robot.enable_power(True)
                robot.enable_control(True)
        elif msg.cmd == msg.STOP_GAME:
            self.game_over = True
            for robot in self.robots.values():
                robot.enable_power(False)
        elif msg.cmd == msg.KILL_ROBOT:
            if msg.robot_name in self.robots.keys():
                robot = self.robots[msg.robot_name]
                robot.enable_power(False)
                robot.remain_hp = 0
                robot.survive = False
        elif msg.cmd == msg.REVIVE_ROBOT:
            if msg.robot_name in self.robots.keys():
                robot = self.robots[msg.robot_name]
                robot.enable_power(True)
                robot.remain_hp = robot.max_hp
                robot.survive = True

def main(args=None):
    rclpy.init(args=args)
    node = Node('referee_system')
    referee_system = SimpleRefereeSystem(node)
    rclpy.spin(node)
    referee_system
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()