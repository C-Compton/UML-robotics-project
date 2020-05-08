#!/usr/bin/env python

import rospy

import math
import threading

from duckietown_msgs.msg import LanePose, Twist2DStamped, BoolStamped
from sign_reader.msg import SignInfo
from utilities.utils import PidController
from enum import Enum

_NODE_NAME = 'arbiter_node'

_STOP_DISTANCE=0.6 # the width of one lane



_LOW_SPD = 0.1
_MED_SPD = 0.2
_HI_SPD  = 0.5
_SPD_TH  = 0.01

class THRES_LEFT(Enum):
    D = 0.25
    PHI = 0.60

class THRES_RIGHT(Enum):
    D = 0.10
    PHI = 0.40

class Arbiter:
    
    def __init__(self):
        # Intercept the original message
        rospy.Subscriber('lane_controller_node/car_cmd', Twist2DStamped, self.checkCarCmd)
        rospy.Subscriber('lane_filter_node/lane_pose', LanePose, self.checkLanePose)
        rospy.Subscriber('sign_reader_node/sign_info', SignInfo, self.checkSign)
        rospy.Subscriber('arbiter_node/new_car_cmd', Twist2DStamped, self.updateSelfState)
        self.pub = rospy.Publisher('arbiter_node/new_car_cmd', Twist2DStamped, queue_size=10)
        self.pubSwitch = rospy.Publisher('apriltag_detector_node/switch', BoolStamped, queue_size=10)

        self.blocking = False
        self.d_phi_oor = False
        
        self.d_err = 0.0
        self.phi_err = 0.0

        self.speed_limit = _MED_SPD
        self.heading=0 # Twist2DStamped omega

        self.vMultiplier = 1
        self.stopCtrl = PidController(0.5, 0, 0, rospy.get_time(), 0, 0)

        self.did_see_sign = False
        self.last_sign = 'NONE'

    def updateSelfState(self, selfState):
        self.curr_v = selfState.v
        self.curr_omega = selfState.omega

    def publish(self, v, o):
        output = Twist2DStamped()
        output.v = min(v, self.speed_limit)
        output.omega = o
        self.pub.publish(output)
        
    def checkCarCmd(self, car_cmd_baseline):
        # If there's no sign to handle, use LF
        if not self.did_see_sign or self.last_sign is 'GO':
            v = car_cmd_baseline.v
            o = car_cmd_baseline.omega
            self.publish(v, o)
        else:
            self.arbitration()

    def checkSign(self, sign_msg):
        sign = sign_msg.sign
        self.dist_to_sign = sign_msg.dist_to_sign

        if sign is not 'NONE':
            self.did_see_sign = True
            self.last_sign = sign

    
    def checkLanePose(self, lanePose_msg):
        self.d_err = lanePose_msg.d
        self.phi_err = lanePose_msg.phi    

    def toggleDidSeeSign(self):
        self.did_see_sign = False
        
    def gentleStop(self):
        if self.curr_v <= _SPD_TH:
            self.publish(0.0, 0.0)
            rospy.Timer(rospy.Duration(3), self.toggleDidSeeSign())
            return

        error_dist = self.dist_to_sign - _STOP_DISTANCE # calculate error
        curr_time = rospy.get_time()

        self.stopCtrl.calibrateTime(curr_time - 0.0001)
        
        v  = self.stopCtrl.pid( curr_time, error_dist)
        rospy.logwarn("gentleStop : " + str(v))
        self.publish(v, self.curr_omega)

    def slowDown(self):
        self.speed_limit = _LOW_SPD

    def speedUp(self):
        self.speed_limit = _MED_SPD

    def moveRobot(self, v, omega, duration_time, rate=100):
        # Generic function for moving the robot for a certain amount of time
        start_time = rospy.get_time()  
        while (rospy.get_time() - start_time) < duration_time:
            rate = rospy.Rate(rate)
            self.publish(v, omega)
            rate.sleep()

        output.v = 0
        output.omega = 0
        self.publish(0, 0)

    def turn(self):
        # Assume that the robot comes to a stop before executing a turn.
        # Use the phi value to center it
        if math.fabs(self.phi_err) > 0.2: 
            self.moveRobot(0, -2 * self.phi_err, 0.5)

        # Then, based on the sign, we turn
        # We set more or less aggressive omegas based on the position of the 
        # robot in the lane
        if self.last_sign == 'LEFT':
            if math.fabs(self.d_err) <= 0.05:
                moveRobot(0.25, 2.2, 3)
            elif self.d_err > 0.05:
                moveRobot(0.25, 1.9, 3)
            elif self.d_err < -0.05:
                moveRobot(0.25, 2.5, 3)

        elif self.last_sign == 'RIGHT':
            if math.fabs(self.d_err) <= 0.05:
                moveRobot(0.25, -3.8, 1.5)
            elif self.d_err > 0.05:
                moveRobot(0.25, -3.9, 1.5)
            elif self.d_err < -0.05:
                moveRobot(0.25, -3.5, 1.5)

        # Comment from Saba:
        # Go straight for 1 sec just to make sure we are in the
        # secound street.(it can be omited because if we set a
        # controler we do n$

    def go(self):
        v = self.curr_v * self.vMultiplier
        omega = self.heading
        self.publish(v, omega)

    def arbitration(self):
        if not self.blocking:
            if self.last_sign == 'STOP':
                self.blocking = True
                self.gentleStop()
                self.blocking = False
                return

            elif self.last_sign == 'LEFT':
                rospy.loginfo('turning left')
                self.blocking = True
                rospy.loginfo('about to stop')
                self.gentleStop()
                rospy.loginfo('about to turn')
                self.turn()
                rospy.loginfo('turn complete')
                self.blocking = False

            elif self.last_sign == 'RIGHT':
                self.blocking = True
                self.gentleStop()
                self.turn()
                self.blocking = False

            elif self.last_sign == 'SLOW':
                self.vMultiplier = 0.5
                self.go()

            elif self.last_sign == 'FAST':
                self.vMultiplier = 1
                self.go()

            self.did_see_sign = False

if __name__=='__main__':
    try:
        rospy.init_node(_NODE_NAME, anonymous=True)
        Arbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TODO : Log Exception thrown")        
