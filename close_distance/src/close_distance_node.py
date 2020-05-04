#!/usr/bin/env python

###############################################################################
# The purpose of this node is to take April tag detections from sign_info and
# maneuver the robot to a set distance and angle relative to the detected
# tag.
###############################################################################

import rospy
import math
from apriltags2_ros.msg import AprilTagDetectionArray
from sign_reader.msg import SignInfo
from sign_reader.signInfo import signDict
import time
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class PID:
	def __init__(self):
		# Subscribe to the sign information that gets me distance and angle.
		rospy.Subscriber('/sign_info',SignInfo,self.adjust_motion)

		# Must include a velocity and angle, like the Twist2DStamped.
		self.pub = rospy.Publisher('horriblegoose/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)

		# Initialize timing and measurements.
		self.previous_time = time.time()
		self.current_time=self.previous_time+0.001
		self.elapsed_time = self.current_time - self.previous_time
		self.last_measurement=0
		self.current_measurement=0
		# Distance controller - comes from dist_to_sign
		self.dp = 0
		self.di = 0
		self.dd=0



	def adjust_motion(self,data):
		twist=Twist2DStamped()
		twist.v=0
		twist.omega=0
		print(data)
		print(data.dist_to_sign)
		if data.dist_to_sign > 0 : # If tag was detected
	
	# Here's some code that acts on ALL detected April tags: maneuvering to be a set
	# distance and angle from the sign. All sign-handling nodes can use this.
			last_measurement=self.current_measurement
			current_measurement = data.dist_to_sign-0.2
			self.current_measurement=current_measurement

			# This will only be published if current_measurement<0.01
			kdp=1
			kdi=0
			kdd=1

			self.dp = current_measurement #data
			self.di = 0 
			self.dd = 0 
			
			twist.v=(kdp*self.dp+kdi*self.di+kdd*self.dd)
			twist.omega=0


			if current_measurement>0.01:
				self.pub.publish(twist)

		#else: # If no tag was detected
		#	twist.v=0
		#	twist.omega=0
		#	self.pub.publish(twist)

if __name__ == '__main__':
	# Get that node started - the launch file will use this
	rospy.init_node('pid_sign',anonymous=True)
	PID()
	rospy.spin()

