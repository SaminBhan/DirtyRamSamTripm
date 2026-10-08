#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node

from std_msgs.msg import Int64, Bool, String
from geometry_msgs.msg import Twist

class ConstantControl(Node): 
	def __init__(self) -> None: 
		# 
		super().__init__("ConstantControl") 

		self.pub_controls = self.create_publisher(String, "/Controls", 10)

		self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)

		self.pub_kill = self.create_publisher(Bool, "/kill", 10)
		
		# self.timer = self.create_timer(0.2, self.callback) 

		self.timer = self.create_timer(0.2, self.vel_callback)

	def callback(self) -> None: 
		msg = String()
		msg.data = "sending constant control" 
		
		self.pub.publish(msg)

	def vel_callback(self) -> None: 
		msg = Twist()
		# go in a circle with radius 5m at 0.5 m/s
		msg.linear.x = 0.5
		msg.angular.z = 0.1
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x= 0.0
		msg.angular.y= 0.0

		self.pub_vel.publish(msg)

	def kill_callback(self) -> None:
		msg = Bool()
		msg.data = True
		self.pub_kill.publish(msg)
		self.pub_vel.publish(0.0)  

if __name__ == "__main__": 
	rclpy.init()
	node = ConstantControl()
	rclpy.spin(node) 
	rclpy.shutdown()
