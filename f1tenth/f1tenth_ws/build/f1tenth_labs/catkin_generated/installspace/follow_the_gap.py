#!/usr/bin/env python2
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

from ros_f110_tools.logger import PeriodicLogger


class GAPFollow:
    """ Implement Follow the Gap algorithm
    """

    def __init__(self):
        # Topics & Subs, Pubs

        self.lookahead_degrees = [80., 280.]
        self.bubble_r = 0.3

        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.get_range)
        self.drive_pub = rospy.Publisher('nav', AckermannDriveStamped, queue_size=10)
        self.debug_pub = rospy.Publisher('debug', String, queue_size=10)

        self.debugger = PeriodicLogger(self.debug_pub, period=1000)

    def get_range(self, data):
        beg = int((self.lookahead_degrees[0]/360.0)*1080.)
        end = int((self.lookahead_degrees[1]/360.0)*1080.)

        ranges = data.ranges[beg:end]

        ranges = np.clip(ranges, 0, 10.0)

        nearest_point = np.min(ranges)
        min_dist = nearest_point + self.bubble_r

        ranges = np.where(ranges <= min_dist, 0.0, ranges)

        gaps = []
        if ranges[0] != 0.0:
            gaps.append(0)

        i = 0
        while i < len(ranges):
            if ranges[i] == 0.0:
                if i > 0:
                    gaps.append(i-1)
                while i < len(ranges) and ranges[i] == 0.0:
                    i += 1
                if i < len(ranges):
                    gaps.append(i)
                continue

            i += 1

        if ranges[-1] != 0.0:
            gaps.append(len(ranges) - 1)

        assert len(gaps) % 2 == 0

        max_gap = -1
        gap_beg = 0
        gap_end = 1080

        # find max gap
        i = 0
        while i < len(gaps):
            if gaps[i+1]-gaps[i] > max_gap:
                max_gap = gaps[i+1]-gaps[i]
                gap_beg = gaps[i]+beg
                gap_end = gaps[i+1]+beg
            i += 2

        mid_point = float(gap_end+gap_beg)/2.
        angle_deg = ((mid_point / 1080.) * 360.) - 180
        angle_rad = angle_deg * (np.pi/180.)

        self.debugger.debug(['angle:'+str(angle_deg), str(mid_point), str(beg), str(gaps), "gap:" + str(gap_beg) + "-" + str(gap_end)])
        angle = angle_rad * 1.0
        self.drive(2., angle)

    def drive(self, speed, angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("GAPFollow_node", anonymous=True)
    gf = GAPFollow()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)