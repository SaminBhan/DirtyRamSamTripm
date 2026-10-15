#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ConstantControl(Node):
	def __init__(self) -> None:
		super().__init__("constant_control")

		self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)

		# subscribe (not publish) to /kill so another terminal can stop us
		self.sub_kill = self.create_subscription(Bool, "/kill", self.kill_callback, 10)

		self.timer = self.create_timer(0.2, self.vel_callback)

	def vel_callback(self) -> None:
		msg = Twist()  # every field starts at 0.0
		# spin in place at 1 rad/s so the stop on /kill is obvious
		msg.linear.x = 0.0
		msg.angular.z = 1.0

		self.pub_vel.publish(msg)

	def kill_callback(self, msg: Bool) -> None:
		if msg.data:
			# cancel first, or the next tick would overwrite the stop command
			self.timer.cancel()
			self.pub_vel.publish(Twist())
			self.get_logger().info("kill received, robot stopped")

if __name__ == "__main__":
	rclpy.init()
	node = ConstantControl()
	rclpy.spin(node)
	rclpy.shutdown()
