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


#if object near the vehicle
#numbering similar to that of clock face 
#immiediate left is 9, immideiate right is 3
ObjectLeft = 9
ObjectRight = 3
ObjectFront = 12
ObjectFrontLeft = 11
ObjectFrontRight = 1
ObjectBackLeft = 7
ObjectBackRight = 5
NoObject = 0

#object away distance default to 15
ObjectLeftDistance = 15
ObjectRightDistance = 15
ObjectFrontDistance = 15
ObjectFrontLeftDistance = 15
ObjectFrontRightDistance = 15
ObjectBackLeftDistance = 15
ObjectBackRightDistance = 15

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

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher(drive_topic_car, AckermannDriveStamped, queue_size = 10) #drive_topic_car
    def DetectSurroundings(self, msg): 
		#msg.ranges should be a length of values representing distance in meters at each point. 
		# 
		# get length of ranges 
		# divide lenght by 7 for each object position 
		# assume 0 index of array is back right 
        length_of_ranges = len(msg.ranges) #Length was 1081
		#array length of each object 
        alo = int(length_of_ranges / 7)
#get minimum distance of each side object range (ie. how close is the closest object 
        ObjectBackRightDistance = (min(msg.ranges[0: alo]))
        ObjectRightDistance = (min(msg.ranges[alo: (alo*2)]))
        ObjectFrontRightDistance = (min(msg.ranges[(alo*2): (alo*3)]))
        ObjectFrontDistance = (min(msg.ranges[(alo*3): (alo*4)]))
        ObjectFrontLeftDistance = (min(msg.ranges[(alo*4): (alo*5)]))
        ObjectLeftDistance = (min(msg.ranges[(alo*5): (alo*6)]))
        ObjectBackLeftDistance = (min(msg.ranges[(alo*6): ((alo*7)-1)]))  	#print(ObjectFrontDistance, msg.ranges[540])
        if ObjectFrontDistance < 1:
            return ObjectFront
        return NoObject

    def scan_callback(self, msg):
        surroundingCheck = self.DetectSurroundings(msg)
        print(surroundingCheck)
        if (surroundingCheck == NoObject):
            print("In if")
            time = int(str(rospy.Time.now()))
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = .500
            self.drive_pub.publish(drive_msg)                   
        else:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 45
            drive_msg.drive.speed = .000
            self.drive_pub.publish(drive_msg)		

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
