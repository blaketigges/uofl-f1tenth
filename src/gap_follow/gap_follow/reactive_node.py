import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        # TODO: Publish to drive
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        proc_ranges = np.clip(ranges, 0, 3)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_gap = 0
        max_gap_start = 0
        max_gap_end = 0
        current_gap = 0
        current_gap_start = 0
        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > 0:
                current_gap += 1
                if current_gap == 1:
                    current_gap_start = i
            else:
                if current_gap > max_gap:
                    max_gap = current_gap
                    max_gap_start = current_gap_start
                    max_gap_end = i - 1
                current_gap = 0
        if current_gap > max_gap:
            max_gap = current_gap
            max_gap_start = current_gap_start
            max_gap_end = len(free_space_ranges) - 1
        if max_gap > 0:
            return max_gap_start, max_gap_end
        else:
            # No gap found
            return None, None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        best_point_index = (start_i + end_i) // 2
        return best_point_index

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR
        closest_point = np.argmin(proc_ranges)
        closest_distance = proc_ranges[closest_point]

        #Eliminate all points inside 'bubble' (set them to zero) 
        bubble_distance = .3
        for i in range(len(proc_ranges)):
            if proc_ranges[i] < bubble_distance:
                proc_ranges[i] = 0
                
        #Find free space
        free_space_ranges = np.zeros(len(proc_ranges))
        for i in range(len(proc_ranges)):
            if proc_ranges[i] > bubble_distance:
                free_space_ranges[i] = proc_ranges[i]
            else:
                free_space_ranges[i] = 0

        #Find max gap
        start_i, end_i = self.find_max_gap(free_space_ranges)
        if start_i is None or end_i is None:
            # No gap found
            return
        
        #Find best point in the gap
        best_point_index = self.find_best_point(start_i, end_i, proc_ranges)
        best_point_distance = proc_ranges[best_point_index]
        best_point_angle = data.angle_min + best_point_index * data.angle_increment

        # Calculate steering angle
        steering_angle = best_point_angle

        # Calculate speed
        speed = 1.0
        if best_point_distance < 0.5:
            speed = 0.5
        elif best_point_distance < 1.0:
            speed = 1.0
        else:
            speed = 2.0

        # Create AckermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed

        # Publish message
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()