import rospy
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np


class TTCEBreak:
	def __init__(self):
		rospy.init_node('ttcnode', anonymous=True)
		self.publisher = rospy.Publisher('ttc', String, queue_size=10)
		self.break_com_pub = rospy.Publisher('brake', AckermannDriveStamped, queue_size=10)
		self.break_mux_pub = rospy.Publisher('brake_bool', Bool, queue_size=10)

		self.prev_collect_time = -1
		self.laser_vals = []
		self.v_vals = []
		self.orient_vals = []
		self.group_t = 0.2

	def reset_states(self, t):
		self.prev_collect_time = t
		self.laser_vals = []
		self.v_vals = []
		self.orient_vals = []

	def check_and_break(self, ttc):

		if ttc < 2.0:
			break_msg = AckermannDriveStamped()
			break_msg.drive.speed = 0.0
			break_msg.drive.acceleration = 0.0
			self.publisher.publish("BREAK")
			self.break_com_pub.publish(break_msg)
			self.break_mux_pub.publish(True)


	def measure_ttc(self):
		if len(self.laser_vals) == 0 or len(self.v_vals) == 0 or len(self.orient_vals) == 0:
			return
		r = np.mean(self.laser_vals)
		v = np.mean(self.v_vals)
		theta = np.mean(self.orient_vals)
		rdot = v*np.cos(theta)

		denominator = max(rdot, 0) + 0.000001
		ttc = r/denominator
		self.check_and_break(ttc)
		self.publisher.publish("ttc:"+str(ttc))

	def avg_dist(self, data):
		
		t = data.header.stamp
		t = t.secs + t.nsecs*1e-9

		if self.prev_collect_time < self.group_t:
			self.prev_collect_time = t
			return

		if t - self.prev_collect_time > 1:
			self.measure_ttc()
			self.reset_states(t)
		else:
			mid = len(data.ranges)//2
			val = (data.ranges[mid] + data.ranges[mid-1] + data.ranges[mid+1])/3
			self.laser_vals.append(val)



	def avg_vx(self, data):
		
		t = data.header.stamp
		t = t.secs + t.nsecs*1e-9

		if self.prev_collect_time < 0:
			self.prev_collect_time = t
			return

		if t - self.prev_collect_time > self.group_t:
			self.measure_ttc()
			self.reset_states(t)
		else:
			self.orient_vals.append(data.pose.pose.orientation.z)
			self.v_vals.append(data.twist.twist.linear.x)

	def run(self):
		rospy.Subscriber('scan', LaserScan, self.avg_dist)
		rospy.Subscriber('odom', Odometry, self.avg_vx)

		rospy.spin()


if __name__ == "__main__":
	ttc_break = TTCEBreak()
	ttc_break.run()