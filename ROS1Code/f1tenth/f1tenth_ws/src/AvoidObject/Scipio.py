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


#default Drive speed 
DefaultDriveSpeed = 0.5
#Max Drive Speed 
MaxSpeed = 1.5
#PreviousSpeed 
PrevSpeed = 0.5

AccelerationSpeed = .1 
# version 2.0


#Constants
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
#ObjectLeftDistance = 15
#ObjectRightDistance = 15
#ObjectFrontDistance = 15
#ObjectFrontLeftDistance = 15
#ObjectFrontRightDistance = 15
##ObjectBackLeftDistance = 15
#ObjectBackRightDistance = 15

#stopping distance to check 
StoppingDistance = .85
SlowDownDistance = 1.25
GoodToAccelerate = 1.5
TurnRight = 0
TurnLeft = 1
TurnSlightRight = 2 
TurnSlightLeft = 3
TurnAround = 6

class Surroundings: 
    #contains all the surroundings detected by the lidar 
    ObjectLeftDistance = 15
    ObjectRightDistance =15
    ObjectFrontDistance =15
    ObjectFrontLeftDistance=15 
    ObjectFrontRightDistance  =15 
    ObjectBackLeftDistance =15
    ObjectBackRightDistance =15
    


class WallFollow:
    """ Implement Wall Following on the car
    """
    PreviousTime = 0
    CurSpeed = 0.5
    def __init__(self):
        global prev_time
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic_simulator = '/nav'
        drive_topic_car = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        prev_time = rospy.get_time()
        self.PreviousTime = rospy.get_time()

        #consider passing all the surrondings as an object
        
        
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.mapping_callback)
        #note drive_topic_simulator publishes to the simulator and nothing will happen in real life 
        #drive_topic_simulator for simulator 
        #drive_topic_car for real life
        self.drive_pub = rospy.Publisher(drive_topic_car, AckermannDriveStamped, queue_size = 10) #drive_topic_car

## set the surrounding object data 
    def MapArea(self, surrounding, msg):
        length_of_ranges = len(msg.ranges) #Length was 1081
		#array length of each object 
        alo = int(length_of_ranges / 7)
        surrounding.ObjectBackRightDistance = (min(msg.ranges[0: alo]))
        surrounding.ObjectRightDistance = (min(msg.ranges[alo: (alo*2)]))
        surrounding.ObjectFrontRightDistance = (min(msg.ranges[(alo*2): (alo*3)]))
        surrounding.ObjectFrontDistance = (min(msg.ranges[(alo*3): (alo*4)]))
        surrounding.ObjectFrontLeftDistance = (min(msg.ranges[(alo*4): (alo*5)]))
        surrounding.ObjectLeftDistance = (min(msg.ranges[(alo*5): (alo*6)]))
        surrounding.ObjectBackLeftDistance = (min(msg.ranges[(alo*6): ((alo*7)-1)]))  
        print("Mapped Area")
        return surrounding 


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


##decide to turn left, turn right, or turn around 
    def TurningDecision(self,surrounding):
        if (surrounding.ObjectFrontRightDistance > surrounding.ObjectFrontLeftDistance):
            if (surrounding.ObjectFrontRightDistance > StoppingDistance):
                return TurnSlightRight
        if (surrounding.ObjectFrontLeftDistance > surrounding.ObjectFrontRightDistance):
            if (surrounding.ObjectFrontLeftDistance > StoppingDistance):
                return TurnSlightLeft
            
  
        if (surrounding.ObjectRightDistance > surrounding.ObjectLeftDistance):
            if (surrounding.ObjectRightDistance > StoppingDistance):
                return TurnRight

        if (surrounding.ObjectRightDistance < surrounding.ObjectLeftDistance):
            if (surrounding.ObjectLeftDistance > StoppingDistance):
                return TurnLeft

        return TurnAround

    def Accelerate (self, targetspeed):
        if (self.CurSpeed <= targetspeed):
            self.CurSpeed += .1
            return self.CurSpeed
        """if (self.CurSpeed <= targetspeed):
            
            current_time = rospy.get_time()
        
            change_time = current_time - prev_time
        
            change_vel = change_time * AccelerationSpeed
            velocity = change_vel + self.CurSpeed
            if (velocity > targetspeed):
                return targetspeed
            return velocity"""
        return self.CurSpeed
    

    def Deaccelerate (self, targetspeed):
        if self.CurSpeed < DefaultDriveSpeed:
            return DefaultDriveSpeed
        if (self.CurSpeed >= targetspeed):
            self.CurSpeed -= .1
            return self.CurSpeed
        """if (self.CurSpeed >= targetspeed):
            current_time = rospy.get_time()
        
            change_time = current_time - prev_time
        
            change_vel = change_time * AccelerationSpeed
            velocity = self.CurSpeed - change_vel
            if velocity < DefaultDriveSpeed:
                return DefaultDriveSpeed
            return velocity"""
        return self.CurSpeed


    #Used to drive via maparea function 
    def mapping_callback(self,msg):
        print("StartingCallback:")
        surrounding = Surroundings()
        surrounding = self.MapArea(surrounding,msg)
        print(surrounding.ObjectFrontDistance)
        

        if (surrounding.ObjectFrontDistance > GoodToAccelerate ):
            print("Driving straight Good To Accelerate.  Closest Object in front:")
            print(surrounding.ObjectFrontDistance )

            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0 

            self.CurSpeed = self.Accelerate(MaxSpeed)
            print("Current Speed: ", self.CurSpeed)
            drive_msg.drive.speed = self.CurSpeed
            self.drive_pub.publish(drive_msg)   
        if (GoodToAccelerate > surrounding.ObjectFrontDistance):
            if(surrounding.ObjectFrontDistance > SlowDownDistance):
                print("Driving straight Maintain Speed.  Closest Object in front:")
                print(surrounding.ObjectFrontDistance )

                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = rospy.Time.now()
                drive_msg.header.frame_id = "laser"
                drive_msg.drive.steering_angle = 0 

                self.CurSpeed = MaxSpeed
                print("Current Speed: ", self.CurSpeed)
                drive_msg.drive.speed = self.CurSpeed
                self.drive_pub.publish(drive_msg)   
        
        if(SlowDownDistance > surrounding.ObjectFrontDistance ):
            if (surrounding.ObjectFrontDistance > StoppingDistance ):
                print("Driving straight Better Slow Down.  Closest Object in front:")
                print(surrounding.ObjectFrontDistance )
      
                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = rospy.Time.now()
                drive_msg.header.frame_id = "laser"
                drive_msg.drive.steering_angle = 0 
                self.CurSpeed = self.Deaccelerate(DefaultDriveSpeed)
                print("Current Speed: ", self.CurSpeed)

                drive_msg.drive.speed = self.CurSpeed
                self.drive_pub.publish(drive_msg)   
            else: 
     

                turn = self.TurningDecision(surrounding)

                if(turn == TurnRight):
                    print("Turning Right, Closest object to Right:")

                    print(surrounding.ObjectRightDistance)

                    drive_msg = AckermannDriveStamped()
                    drive_msg.header.stamp = rospy.Time.now()
                    drive_msg.header.frame_id = "laser"
                    drive_msg.drive.steering_angle = -45
                    drive_msg.drive.speed = .500
                    self.drive_pub.publish(drive_msg)		
                if (turn == TurnLeft):
                    print("Turning Left, Closest object to Left:")

                    print(surrounding.ObjectLeftDistance)

                    drive_msg = AckermannDriveStamped()
                    drive_msg.header.stamp = rospy.Time.now()
                    drive_msg.header.frame_id = "laser"
                    drive_msg.drive.steering_angle = 45
                    drive_msg.drive.speed = .500
                    self.drive_pub.publish(drive_msg)	
                if (turn == TurnSlightRight):
                    print("Turning Right, Closest object to Right:")

                    print(surrounding.ObjectFrontRightDistance)

                    drive_msg = AckermannDriveStamped()
                    drive_msg.header.stamp = rospy.Time.now()
                    drive_msg.header.frame_id = "laser"
                    drive_msg.drive.steering_angle = -22.5
                    drive_msg.drive.speed = .500
                    self.drive_pub.publish(drive_msg)	
                if (turn == TurnSlightLeft):
                    print("Turning slight left, Closest object to Left:")

                    print(surrounding.ObjectFrontLeftDistance)

                    drive_msg = AckermannDriveStamped()
                    drive_msg.header.stamp = rospy.Time.now()
                    drive_msg.header.frame_id = "laser"
                    drive_msg.drive.steering_angle = 22.5
                    drive_msg.drive.speed = .500
                    self.drive_pub.publish(drive_msg)	
                if (turn == TurnAround):
                #add logic to turn around 
                    drive_msg = AckermannDriveStamped()
                    drive_msg.header.stamp = rospy.Time.now()
                    drive_msg.header.frame_id = "laser"
                    drive_msg.drive.steering_angle = 0
                    drive_msg.drive.speed = .000
                    self.drive_pub.publish(drive_msg)	
                    print("Turn Around!")
                    print("Closest object to Right:")

                    print(surrounding.ObjectRightDistance)
                    print("Closest object to Left:")

                    print(surrounding.ObjectLeftDistance)
                    print(" Closest Object in front:")
                    print(surrounding.ObjectFrontDistance )
        
        self.PreviousTime = rospy.get_time()
        print("FinishCallback:")

            
             


#used if you want to drive with surrounding check function 
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
            drive_msg.drive.steering_angle = -45
            drive_msg.drive.speed = .000
            self.drive_pub.publish(drive_msg)		

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
