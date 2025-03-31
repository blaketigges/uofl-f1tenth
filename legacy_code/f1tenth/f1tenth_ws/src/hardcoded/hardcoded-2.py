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
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
from nav_msgs.msg import Odometry


class WallFollow:
    """ Implement Wall Following on the car
    """

    def __init__(self):
        self.done_accelerating = False
        self.current_rpm = 0.0
        self.current_odom = '0'
        global prev_timerostopic 
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic_simulator = '/nav'
        drive_topic_car = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        prev_time = rospy.get_time()

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.pid_control)
        self.drive_pub = rospy.Publisher(drive_topic_car, AckermannDriveStamped, queue_size = 10) #drive_topic_car
        self.rpm_sub =  rospy.Subscriber('/vesc/commands/motor/speed', Float64, self.check_rpm)
        self.odom_sub = rospy.Subscriber('/vesc/odom', Odometry, self.check_odom)

    def check_odom(self, msg):
        self.current_odom = msg

    def check_rpm(self, msg):
        self.current_rpm = msg.data

    def turn_right(self,speed,seconds):
        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+(650000000): #about 2.75 seconds from a stop at 0.5
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = -45
            drive_msg.drive.speed = 3
            self.drive_pub.publish(drive_msg)

    def turn_left(self,speed,seconds):
        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+(520000000): 
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = 45
            drive_msg.drive.speed = 3
            self.drive_pub.publish(drive_msg)

    def constant_forward(self, speed, seconds):
        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+(seconds*1000000000):
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = speed #speed
            self.drive_pub.publish(drive_msg)
            print(self.current_rpm)
        print("Time", (int(str(rospy.Time.now())) - time)/1000000000)

    def car_wait(self, speed, seconds):
        time = int(str(rospy.Time.now()))
        time_to_stop = 0
        odom_msg = '0'
        while int(str(rospy.Time.now())) < time+(1000000000*seconds):
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = 0.0 
            self.drive_pub.publish(drive_msg)
            if(self.current_rpm > 0):
                time_to_stop = (int(str(rospy.Time.now())) - time)/1000000000
        print(time_to_stop)


    def accelerate(self, initialSpeed, targetSpeed, seconds): #acceleration is current (final speed - initial speed)/time
        acceleration = (targetSpeed - initialSpeed) / seconds
        newtime = 0
        oldtime = 0
        time= int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+(1000000000*seconds):
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = 0
            newtime = int(str(rospy.Time.now()))
            drive_msg.drive.speed = initialSpeed + ((newtime-time)*acceleration)/1000000000
            oldtime = newtime
            self.drive_pub.publish(drive_msg)
        drive_msg.drive.speed = targetSpeed

    def five_meter_sprint(self):
        input("press enter to continue")
        self.constant_forward(1, 5)
        input("press enter to continue")
        self.constant_forward(2, 2.5)
        input("press enter to continue")
        self.constant_forward(3, 5/3)
        input("press enter to continue")
        self.constant_forward(4, 5/4)
        input("press enter to continue")
        self.constant_forward(5, 1)
        input("DONT PRESS ENTER AGAIN just exit the program")

    def check_acceleration(self, speed): #function that is meant to time how fast we can get to our desired rpm
        time_init = 0
        input("press enter to test acceleration")
        time = int(str(rospy.Time.now()))
        while self.current_rpm < 4614*speed: # Until it about reaches its acceleration
            if self.current_rpm < 1300:
                time_init = (int(str(rospy.Time.now())) - time)
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = speed #speed
            self.drive_pub.publish(drive_msg)
        print(self.current_rpm)
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = 0
        drive_msg.drive.speed = speed #speed
        self.drive_pub.publish(drive_msg)
        print(self.current_rpm)
        print("Time_init", time_init/1000000000)
        print("Time", (int(str(rospy.Time.now())) - time)/1000000000)

    def test_acceleration(self, initialSpeed, targetSpeed, seconds): #hopefully will back itself back up to when acceleration stopped.
        acceleration = (targetSpeed - initialSpeed) / seconds
        newtime = 0
        oldtime = 0
        time= int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+(1000000000*seconds):
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = 0
            newtime = int(str(rospy.Time.now()))
            drive_msg.drive.speed = initialSpeed + ((newtime-time)*acceleration)/1000000000
            oldtime = newtime
            self.drive_pub.publish(drive_msg)
        drive_msg.drive.speed = targetSpeed
        xpos = self.current_odom.pose.pose.position.x
        ypos = self.current_odom.pose.pose.position.y
        #print("Time", (int(str(rospy.Time.now())) - time)/1000000000)
        #print("RPM", (self.current_rpm))
        finish_time = (int(str(rospy.Time.now())) - time)/1000000000
        finish_rpm = self.current_rpm
        self.car_wait(0, 1)
        while(self.current_odom.pose.pose.position.x > xpos):
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = 0  
            drive_msg.drive.speed = -0.3
            self.drive_pub.publish(drive_msg)
        print("Measurement should be (in inches):",  (0.5)*(acceleration)*(seconds**2)*39.37)
        print("Measurement may be (in inches):",  (0.5)*(finish_rpm/4614/(seconds-0.06))*((seconds-0.06)**2)*39.37)
        self.car_wait(0, 1)

    def test_speed(self, speed, seconds):
        xpos_init = self.current_odom.pose.pose.position.x
        time = int(str(rospy.Time.now()))
        time_init = 0
        time_to_accelerate = 0
        while int(str(rospy.Time.now())) < time+(seconds*1000000000):
            if self.current_rpm < 1300:
                time_init = (int(str(rospy.Time.now())) - time)
            elif self.current_rpm < speed*4614-1:
                time_to_accelerate = (int(str(rospy.Time.now()))) - time - time_init
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = speed #speed
            self.drive_pub.publish(drive_msg)
            print(self.current_rpm)
            new_xpos = self.current_odom.pose.pose.position.x
        print("Time", (int(str(rospy.Time.now())) - time)/1000000000)
        print("Time taken to start rolling", time_init/1000000000)
        print("Time taken to accelerate", time_to_accelerate/1000000000)
        print("Distance traveled (in car units):", new_xpos-xpos_init)
        print("Measurement should be (in inches):",  (speed*(seconds-(time_init/1000000000))*39.37))
        self.car_wait(0, 1)
        while(self.current_odom.pose.pose.position.x > new_xpos):
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = 0  
            drive_msg.drive.speed = -0.3
            self.drive_pub.publish(drive_msg)
        self.car_wait(0, 1)

##################################################################
    def pid_control(self, error):
        #input("press enter to continue")
        #self.accelerate(0, 3, 2.5)
        #self.turn_right(1,1)
        #self.constant_forward(3, 0.5)
        #self.turn_right(1,1)
        #self.constant_forward(3, 0.6)
        #self.turn_left(1,1)
        #self.constant_forward(3, 0.5)
        #self.turn_left(1,1)
        #self.constant_forward(3, 0.6)
        #self.car_wait(0, 5)
        #self.constant_forward(2, 1.5)

        self.test_speed(0.5, 5)
        input("press enter to continue")
        self.test_speed(0.5, 5)
        input("press enter to continue")
        self.test_speed(0.5, 5)
        input("press enter to continue")
        self.test_speed(0.5, 5)
        input("press enter to continue")
        self.test_speed(0.5, 5)
        input("press enter to continue")
        input("press enter to leave testing")

#        #self.check_acceleration(5) 
#        input("press enter to continue")
#        self.test_acceleration(0, 1, 2)
#        input("press enter to continue")
#        self.test_acceleration(0, 2, 2)
#        input("press enter to continue")
#        self.test_acceleration(0, 3, 2)
#        input("press enter to continue")
#        self.test_acceleration(0, 4, 2)
#        input("press enter to continue")
#        self.test_acceleration(0, 5, 2)

        #self.turn_left(1,1)
        #self.turn_left(1,1)
#        self.accelerate(0, 1.5, 3)
#        self.accelerate(1.5, 0, 1)
#        self.constant_forward(5, 3)
#        self.turn_left(0.5, 2720000000)
#        self.turn_right(0.5, 2740000000)
#        self.car_wait(0, 5)

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
        while int(str(rospy.Time.now())) < time+2000000000:
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = 0.0
            self.drive_pub.publish(drive_msg)

        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+2720000000:
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = -45
            drive_msg.drive.speed = 0.5
            self.drive_pub.publish(drive_msg)

        time = int(str(rospy.Time.now()))
        while int(str(rospy.Time.now())) < time+2000000000:
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser1"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = 0.0
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
