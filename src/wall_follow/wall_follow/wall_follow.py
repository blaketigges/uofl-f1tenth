import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        self.kp = 0.340
        self.kd = 0.005
        self.ki = 0.001

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.servo_offset = 0.0
        self.prev_time = 0.0

        # TODO: store any necessary values you think you'll need

        #WALL FOLLOW PARAMS
        self.ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
        self.DESIRED_DISTANCE_RIGHT = 0.9 # meters
        self.DESIRED_DISTANCE_LEFT = 0.85
        self.VELOCITY = 5.5 # meters per second
        self.CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        if angle >= -45 and angle <= 225:
            iterator = len(range_data) * (angle + 90) / 360
            if not np.isnan(range_data[int(iterator)]) and not np.isinf(range_data[int(iterator)]):
                return range_data[int(iterator)]


    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        left_angle = 90.0
        left_dist = self.get_range(range_data, left_angle)
        if left_dist is None:
            left_dist = dist
        error = dist - left_dist
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        current_time = self.get_clock().now().to_msg().nanosec / 1e9
        del_time = current_time - self.prev_time
        self.integral += self.prev_error * del_time
        angle = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error) / del_time
        self.prev_error = error
        self.prev_time = current_time

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle
        if abs(angle) > np.radians(0) and abs(angle) <= np.radians(10):
            drive_msg.drive.speed = velocity
        elif abs(angle) > np.radians(10) and abs (angle) <= np.radians(20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5
        self.publisher.publish(drive_msg)

    def followLeft(self, data, leftDist):
        front_scan_angle = 125
        back_scan_angle = 180
        teta = math.radians(abs(front_scan_angle - back_scan_angle))
        front_scan_dist = self.get_range(data, front_scan_angle)
        back_scan_dist = self.get_range(data, back_scan_angle)
        alpha = math.atan2(front_scan_dist * math.cos(teta) - back_scan_dist, front_scan_dist * math.sin(teta))
        wall_dist = back_scan_dist * math.cos(alpha)
        ahead_wall_dist = wall_dist + self.CAR_LENGTH * math.sin(alpha)
        return leftDist - ahead_wall_dist
    
    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # error = self.get_error(msg.ranges, self.DESIRED_DISTANCE_RIGHT)
        error = self.followLeft(msg.ranges, self.DESIRED_DISTANCE_LEFT)
        velocity = self.VELOCITY
        if abs(error) > 0.2:
            velocity = self.VELOCITY / 2
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()