#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # TODO: create ROS subscribers and publishers.

        lidarscan_topic = '/scan'
        odom_topic = '/ego_racecar/odom' #  the longitudinal velocity of the vehicle can be found in twist.twist.linear.x
        drive_topic = '/drive'
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)




    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        valid_ranges = np.where(np.isfinite(ranges), ranges, np.inf)
        threshold = 1

        # Avoid division by zero and handle large values
        if self.speed > 0:
            # Perform division only on finite values
            ittc = valid_ranges / self.speed
            ittc = np.where(valid_ranges == np.inf, np.inf, ittc)

            # Check if the minimum TTC is below the threshold
            if np.min(ittc) < threshold:
                print("TTC is less than threshold, emergency braking!")
                drive_msg = AckermannDriveStamped()
                drive_msg.drive.speed = 0.0
                self.publisher.publish(drive_msg)
        
        # TODO: publish command to brake
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()