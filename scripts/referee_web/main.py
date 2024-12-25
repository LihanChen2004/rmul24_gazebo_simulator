#!/usr/bin/python3

# =====================================================================  libs   ==========================================

# ==========================================
#    flask
# ==========================================
import logging
import sys
import threading
import time
from threading import Lock

# ==========================================
#   packages for ros
# ==========================================
import rclpy
from engineio.payload import Payload
from flask import Flask, render_template, request
from flask_cors import CORS
from flask_socketio import Namespace, SocketIO, emit

from rmoss_interfaces.msg import RefereeCmd, RobotStatus

# ==========================================
#    robot list
# ==========================================
robot_names = ["red_standard_robot1", "blue_standard_robot1"]
node = None

# ==========================================
#    cmd constant
# ==========================================
control_map = {
    "PREPARATION": 0,  # system cmd: 3 min preparation (free control for robot, disable referee system)
    "SELF_CHECKING": 1,  # system cmd: referee system self-checking (unable to control robot, and init all referee system data).
    "START_GAME": 2,  # system cmd: start game (free control for robot, enable referee system)
    "STOP_GAME": 3,  # system cmd: stop game  (unable to control robot, disable referee system)
    "KILL_ROBOT": 4,  # control robot: kill robot with robot_name
    "REVIVE_ROBOT": 5,  # control robot: revive robot with robot_name
}

# ==========================================
#    config
# ==========================================
async_mode = "threading"
Payload.max_decode_packets = 10000
app = Flask(__name__)
app.config["SECRET_KEY"] = "secret!"
CORS(app)
socketio = SocketIO(app, async_mode=async_mode)
info_thread = None
thread_lock = Lock()
log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

# =====================================================================  Classes   ==========================================


class Timer:
    def __init__(self):
        self.start_time = None
        self.elapsed_time = 0
        self.running = False

    def start(self):
        if not self.running:
            self.start_time = time.time() - self.elapsed_time
            self.running = True

    def stop(self):
        if self.running:
            self.elapsed_time = time.time() - self.start_time
            self.running = False

    def reset(self):
        self.start_time = None
        self.elapsed_time = 0
        self.running = False

    def get_time(self):
        if self.running:
            return time.time() - self.start_time
        else:
            return self.elapsed_time


# ==========================================
#    namespace class
# ==========================================
class RefereeSocketHandler(Namespace):
    def __init__(self, namespace):
        super().__init__(namespace=namespace)
        self.robot_name = namespace[1:]
        self.info_thread = None

        self.timer = Timer()
        self.timer_thread = threading.Thread(target=self.send_timer_info)
        self.timer_thread.start()

        # ==========================================
        #   pubs
        # ==========================================
        self.referee_pub = node.create_publisher(
            RefereeCmd, "/referee_system/referee_cmd", 10
        )

    def on_connect(self):
        global info_thread, node, robot_names, chosen_robot_dict
        emit("robot_names", {"list": robot_names})
        with thread_lock:
            if info_thread is None:
                info_thread = socketio.start_background_task(ros_info_thread, node)

    def on_control(self, message):
        robot_name = message.get("robot_name", "")
        instruction = message["instruction"]
        if instruction == "START_GAME":
            print("start game timer")
            self.timer.start()
        elif instruction == "STOP_GAME":
            print("stop game timer")
            self.timer.stop()
        elif instruction == "SELF_CHECKING":
            print("reset game timer")
            self.timer.reset()
        cmd = control_map[instruction]
        msg = RefereeCmd()
        msg.cmd = cmd
        msg.robot_name = robot_name
        self.referee_pub.publish(msg)

    def on_default_error_handler(self, e):
        print("======================= ERROR =======================")
        print(e)
        print(request.event["message"])
        print(request.event["args"])
        print("=====================================================")

    def send_timer_info(self):
        while True:
            time.sleep(1)
            if self.timer.running:
                socketio.emit("timer_info", {"time": self.timer.get_time()})


# =============================================================================================================================

# =====================================================================  funcitons   ==========================================


# ==========================================
#    发送血量、弹药量、图像数据给前端
# ==========================================
def send_refere_info_callback(robot_name):
    def func(message):
        # print(message)
        socketio.emit(
            "robot_status",
            {
                "robot_name": robot_name,
                "remain_hp": message.remain_hp,
                "max_hp": message.max_hp,
                "total_projectiles": message.total_projectiles,
                "used_projectiles": message.used_projectiles,
                "hit_projectiles": message.hit_projectiles,
            },
        )

    return func


# ==========================================
#    信息发送线程
# ==========================================
def ros_info_thread(node):
    global BLUE_HP, RED_HP, ATTACK_INFO
    for robot_name in robot_names:
        hp_sub = node.create_subscription(
            RobotStatus,
            "/referee_system/%s/robot_status" % (robot_name),
            send_refere_info_callback(robot_name),
            10,
        )
    rclpy.spin(node)
    node.destroy_node()


# ==========================================
#    返回页面
# ==========================================
@app.route("/")
def index():
    return render_template("index.html", async_mode=socketio.async_mode)


# ============================================================================================================================
# =====================================================================  others   ============================================


# ==========================================
#    init node
# ==========================================
rclpy.init()
node = rclpy.create_node("referee_web")

# ==========================================
#    init namespace class
# ==========================================
socketio.on_namespace(RefereeSocketHandler("/"))

if __name__ == "__main__":
    if len(sys.argv) == 3:
        robot_name = str(sys.argv[1])
        port = int(sys.argv[2])
    socketio.run(app, host="0.0.0.0", port=2350)
