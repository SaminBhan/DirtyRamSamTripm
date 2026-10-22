#!/usr/bin/env python3

import numpy as np
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState

class HeadingController(BaseHeadingController):
    def __init__(self) -> None:
        super().__init__("heading_controller")
        self.kp = 2.0

    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        error = wrap_angle(goal.theta - state.theta)
        omega = self.kp * error
        msg = TurtleBotControl()
        msg.omega = omega
        return msg

if __name__ == "__main__":
    rclpy.init()
    node = HeadingController()
    rclpy.spin(node)
    rclpy.shutdown()


