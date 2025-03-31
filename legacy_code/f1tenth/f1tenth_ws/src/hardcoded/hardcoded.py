#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
from time import sleep

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        global prev_time
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic_simulator = '/nav'
        drive_topic_car = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        prev_time = rospy.get_time()

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.pid_control)
        self.drive_pub = rospy.Publisher(drive_topic_car, AckermannDriveStamped, queue_size = 10) #drive_topic_car

    def pid_control(self, error):
        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+1000000000:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = .500
            self.drive_pub.publish(drive_msg)
        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+3000000000:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = ((int(str(rospy.Time.now())) - time) / 3000000000) + 0.5
            self.drive_pub.publish(drive_msg)
        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+1000000000:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = 1.5
            self.drive_pub.publish(drive_msg)
        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+3000000000:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = 1.5 - ((int(str(rospy.Time.now())) - time) / 3000000000)
            self.drive_pub.publish(drive_msg)
        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+4000000000:
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = 45
            drive_msg.drive.speed = 0.5
            self.drive_pub.publish(drive_msg)
#        time = int(str(rospy.Time.now()))
#        while int(str(rospy.Time.now())) < time+5000000000:
#            drive_msg = AckermannDriveStamped()
#            drive_msg.header.stamp = rospy.Time.now()
#            drive_msg.header.frame_id = "laser"
#            drive_msg.drive.steering_angle = 0
#            drive_msg.drive.speed = 0.75
#            self.drive_pub.publish(drive_msg)
#        time = int(str(rospy.Time.now()))
#        while int(str(rospy.Time.now())) < time+4000000000:
#		print(rospy.Time.now())
#            drive_msg.header.stamp = rospy.Time.now()
#            drive_msg.header.frame_id = "laser"
#            drive_msg.drive.steering_angle = 45
#            drive_msg.drive.speed = 0.5
#            self.drive_pub.publish(drive_msg)
#        sleep(0.5)
#        drive_msg.header.stamp = rospy.Time.now()
#        drive_msg.header.frame_id = "laser"
#        drive_msg.drive.steering_angle = 0
#        drive_msg.drive.speed = 0.5
#        self.drive_pub.publish(drive_msg)
#        sleep(2)	


#    def lidar_callback(self, data):
#        error = self.followLeft(data.ranges, DESIRED_DISTANCE_LEFT) 
#        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
