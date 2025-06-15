import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import tf_transformations

class ParallelParkingNode(Node):
	def __init__(self):
		super().__init__('parallel_parking_node')

		self.ld19_sub = self.create_subscription(
			LaserScan,
			'/scan',
			self.ld19_callback,
			QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
		)
		self.sick_sub = self.create_subscription(
			PointCloud,
			'/cloud',
			self.sick_callback,
			QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
		)
		
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
		
		self.timer = self.create_timer(0.1, self.control_loop)
		self.ld19_data = 0
		self.sick_data = 0
		self.state = "searching"

	def control_loop(self):
		control_vesc = Twist()

		side_window = ld19_ranges[80:100]  # adjust as needed
		side_distances = [r for r in side_window if not math.isnan(r)]
		min_side_distance = min(side_distances) if side_distances else 0.0

		# Parking gap detection threshold
		gap_threshold = 1.0

		if self.state == "searching":
			control_vesc.linear.x = 0.2
			control_vesc.angular.z = 0.0

			if min_side_distance > gap_threshold:
				self.node.get_logger().info(" Parking gap detected!")
				self.state = "align"
				self.timer = 0

		elif self.state == "align":
			control_vesc.linear.x = -0.1
			control_vesc.angular.z = 0.4
			self.timer += 1
			if self.timer > 20:
				self.state = "straighten"
				self.timer = 0

		elif self.state == "straighten":
			control_vesc.linear.x = -0.1
			control_vesc.angular.z = -0.4
			self.timer += 1
			if self.timer > 20:
				self.state = "stop"

		elif self.state == "stop":
			control_vesc.linear.x = 0.0
			control_vesc.angular.z = 0.0
			self.node.get_logger().info("Parked successfully.")

		self.publisher_.publish(control_vesc)

	def ld19_callback(self, msg):
		self.ld19_data = msg.ranges

	def sick_callback(self, msg):
		self.sick_data = msg.points


def main(args=None):
	rclpy.init(args=args)
	node = ParallelParkingNode()

	def signal_handler(sig, frame):
		node.stop_robot()
		node.destroy_node()
		rclpy.shutdown()

	signal.signal(signal.SIGINT, signal_handler)
	rclpy.spin(node)

