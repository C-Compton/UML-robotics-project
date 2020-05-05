#!/usr/bin/env python

import rospy
import math
from apriltags2_ros.msg import AprilTagDetectionArray
from sign_reader.msg import SignInfo
from sign_reader.signInfo import signDict

class SignReader:
    def __init__(self):
        rospy.Subscriber(
            '/duckiebot/tag_detections', 
            AprilTagDetectionArray, 
            self.printer
        )

        self.pubSignInfo = rospy.Publisher('/sign_info', SignInfo, queue_size=10)
        
        self.derivative_x = self.derivative_z = self.integral_x = self.integral_z = self.v_ = \
            self.omega_ = self.prior_integral_x = self.prior_integral_z = self.prior_error_x = self.prior_error_z\
                 = self.error_x = self.error_z = 0.0
	self.last_time = None
	self.current_time  = rospy.get_time()
	self.det_time = 0
        

    def printer(self, aprilTagInfo):
        k_p_x, k_i_x, k_d_x = 1, 0, 0
        k_p_z, k_i_z, k_d_z = 4, 0, 0.6
        obj = Twist2DStamped()
        
        # If the length of the detections list is 0, we aren't seeing any signs
        if len(aprilTagInfo.detections):
            for detection in aprilTagInfo.detections:
                error_z = aprilTagInfo.detections[0].pose.pose.pose.position.error_z
                signId = aprilTagInfo.detections[0].id[0]
                self.pubSignInfo.publish(signDict[signId], error_z)
        else:
            error_z = 0
            self.pubSignInfo.publish(signDict[-1], -1)

	if self.last_time==None:
        	self.last_time=rospy.get_time()

	self.current_time=rospy.get_time()
        self.det_time=self.current_time - self.last_time

        self.derivative_x = (self.error_x - self.prior_error_x) / self.det_time
        self.derviative_z = (self.error_z - self.prior_error_z) / self.det_time
        self.integral_x = self.prior_integral_x + (self.error_x * self.det_time)
        self.prior_integral_x = self.integral_x

        self.integral_z = self.prior_integral_z + (self.error_z * self.det_time)
        self.prior_integral_z = self.integral_z
	self.omega_ = ((self.error_x * k_p_x) + (self.integral_x * k_i_x) + (self.derivative_x * k_d_x))

        self.v_ =  ((self.error_z * k_p_z) + (self.integral_z * k_i_z) + (self.derivative_z * k_d_z))

        self.prior_error_x = self.error_x
        self.prior_error_z = self.error_z

        if signId == signDict["SLOW"]:
            obj.v = self.v_ / 2.0 #Divide by 2 to make it slower

        # elif signId == signDict["FAST]:
        #     obj.v = self.v_ * 1.5 # Multiply by 1.5 to make it faster
        # elif signId == signDict["LEFT"]:
        #     # set some values
        # elif signId == signDict["RIGHT"]:
        #     # set some values

        elif signId == signDict["STOP"]:
            obj.v = self.v_

        obj.omega = self.omega_

        self.pub.publish(obj)
        rospy.logwarn("Velocity : {}, Omega: {}".format(self.v_, self.omega_))

        


if __name__ == '__main__':
    rospy.init_node('sign_reader_node', anonymous=True)
    SignReader()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

