from geometry_msgs.msg import Twist
import math

class ParkingController:
    def __init__(self, node):
        self.node = node
        self.state = "searching"
        self.timer = 0
        self.gap_detected = False

    def compute_control(self, processed_data):
        cmd = Twist()

        if not processed_data["ld19"] or not processed_data["sick"]:
            self.node.get_logger().warn("Waiting for both LiDAR scans...")
            return cmd  # stop if no data

        ld19_ranges = processed_data["ld19"].ranges
        sick_ranges = processed_data["sick"].ranges

        # Example: side LiDAR data from ~90° (facing right side)
        side_window = ld19_ranges[80:100]  # adjust as needed
        side_distances = [r for r in side_window if not math.isnan(r)]
        min_side_distance = min(side_distances) if side_distances else 0.0

        # Parking gap detection threshold
        gap_threshold = 1.0  

        if self.state == "searching":
            cmd.linear.x = 0.2 
            cmd.angular.z = 0.0

            if min_side_distance > gap_threshold:
                self.node.get_logger().info(" Parking gap detected!")
                self.state = "align"
                self.timer = 0

        elif self.state == "align":
            cmd.linear.x = -0.1  
            cmd.angular.z = 0.4  
            self.timer += 1
            if self.timer > 20:
                self.state = "straighten"
                self.timer = 0

        elif self.state == "straighten":
            cmd.linear.x = -0.1
            cmd.angular.z = -0.4 
            self.timer += 1
            if self.timer > 20:
                self.state = "stop"

        elif self.state == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.node.get_logger().info("Parked successfully.")

        return cmd
