#!/usr/bin/python3
import sys
import termios
import tty

import rclpy

from rmoss_interfaces.msg import RefereeCmd

banner = """
This node takes keypresses from the keyboard and publishes them
as RefereeCmd messages.
---------------------------
q : 3 min preparation (reset robot)
w : referee system self-checking (reset referee system)
e ：start game (start referee system)
r ：stop game (stop referee system)
---------------------------
CTRL-C to quit
"""


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node("referee_system_client_test")
    # get params
    pub = node.create_publisher(RefereeCmd, "/referee_system/referee_cmd", 10)
    print(banner)
    cmd = 0
    while True:
        key = getKey(settings)
        if key == "q":
            cmd = 0
        elif key == "w":
            cmd = 1
        elif key == "e":
            cmd = 2
        elif key == "r":
            cmd = 3
        elif key == "\x03":
            break
        msg = RefereeCmd()
        msg.cmd = cmd
        pub.publish(msg)


if __name__ == "__main__":
    main()
