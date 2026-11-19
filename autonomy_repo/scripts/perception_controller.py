#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Bool

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState


class PerceptionController(BaseHeadingController):
    def __init__(self) -> None:
        super().__init__("perception_controller")
        self.declare_parameter("active", True)

        self.image_detected = False
        self.detector_sub = self.create_subscription(
            Bool, "/detector_bool", self.detector_callback, 10
        )

    @property
    def active(self) -> bool:
        return self.get_parameter("active").value

    def detector_callback(self, msg: Bool) -> None:
        if msg.data:
            self.image_detected = True

    def compute_control_with_goal(
        self, state: TurtleBotState, goal: TurtleBotState
    ) -> TurtleBotControl:
        control = TurtleBotControl()
        control.omega = 0.0 if self.image_detected else 0.2
        return control


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
