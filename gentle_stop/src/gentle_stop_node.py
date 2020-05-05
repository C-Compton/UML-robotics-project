#!/usr/bin/env python

import rospy
import time
from duckietown_msgs.msg import LanePose, Twist2DStamped
from sign_reader.msg import SignInfo
from std_msgs.msg import Float32

_STOP_DISTANCE=0.22 # the width of one lane
_THRESHOLD=0.01

class PidController:

    def __init__(self, kp, ki, kd, init_error, init_integ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = init_error
        self.integ = init_integ

    def pid(self, prev_time, curr_time, curr_error):
        dt = prev_time - curr_time
        self.integ += curr_error * dt

        p = curr_error * self.kp
        i = self.integ * self.ki
        d = ((curr_error - self.prev_error) / dt ) * self.kd

        self.prev_error = curr_error
        return  p + i + d



class Stopper:
    
    def __init__(self):
        # Intercept the original message
        rospy.Subscriber('horriblegoose/lane_controller_node/car_cmd', Twist2DStamped, self.checkCarCmd)
        rospy.Subscriber('sign_reader_node/sign_info', SignInfo, self.checkSign)
        self.pub = rospy.Publisher('stop_controller_node/car_cmd', Twist2DStamped, queue_size=10)
        self.debug=rospy.Publisher('pid_output',Float32,queue_size=10)
        self.speed=0 # Twist2DStamped d
        self.heading=0 # Twist2DStamped omega
        self.kd=1
        self.ki=0
        self.kd=0
        self.ctrl=PidController(self.kd,self.ki,self.kd,0,0)

    def checkCarCmd(self,carCmd_baseline):
        if self.did_see_sign == False: 
            self.speed=carCmd_baseline.d
            self.heading=carCmd_baseline.omega

    
    def checkSign(self, sign_msg):
        sign = sign_msg.sign
        self.dist=sign_msg.dist_to_sign
        if sign is 'STOP':

            if self.previous_time is None:
                self.previous_time=time()

            self.current_time=time()

            kp=1
            ki=0
            kd=0
            init_error=self.dist
            if self.integ is None:
                self.integ=0

            output = Twist2DStamped()
            if self.count is None:
                self.count=0
            self.count+=1
            self.debug.publish(self.count)
            #vel=PidController(kp, ki, kd, init_error, self.integ)
            ctrl_output=self.ctrl.pid(self.previous_time, self.current_time,self.dist-_STOP_DISTANCE)
            self.debug.publish(ctrl_output)
            self.previous_time=self.current_time

            if self.speed>_THRESHOLD:
                output.d=self.speed
            else:
                output.d=0
            output.omega=self.heading

            self.pub.publish(output)


            # Do stuff with sign message

 
        

if __name__=='__main__':
    try:
        rospy.init_node("gentle_stop_node", anonymous=True)
        Stopper()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TODO : Log Exception thrown")        
