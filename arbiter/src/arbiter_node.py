#!/usr/bin/env python

import rospy

import math
import threading

from duckietown_msgs.msg import LanePose, Twist2DStamped, BoolStamped
from arbiter.msg import OmegaTune
from sign_reader.msg import SignInfo
from utilities.utils import PidController
from enum import Enum

_NODE_NAME = 'arbiter_node'

_STOP_DISTANCE=0.6 # the width of one lane

_LOW_SPD = 0.1
_MED_SPD = 0.2
_HI_SPD  = 0.5
_SPD_TH  = 0.01

_LEFT_OMEGA = 2.0
_LEFT_DURATION = 1.5

_RIGHT_OMEGA = 4.0
_RIGHT_DURATION = 1.5

class Arbiter:
    
    def __init__(self):
        # Intercept the original message
        rospy.Subscriber('lane_controller_node/car_cmd', Twist2DStamped, self.arbitration)
        rospy.Subscriber('lane_filter_node/lane_pose', LanePose, self.checkLanePose)
        rospy.Subscriber('sign_reader_node/sign_info', SignInfo, self.checkSign)
        rospy.Subscriber('arbiter_node/new_car_cmd', Twist2DStamped, self.updateSelfState)
        rospy.Subscriber('omega_tune/tune', OmegaTune, self.updateTurnOmega)
        self.pub = rospy.Publisher('arbiter_node/new_car_cmd', Twist2DStamped, queue_size=10)
        self.pubSwitch = rospy.Publisher('apriltag_detector_node/switch', BoolStamped, queue_size=10)

        self.blocking = False
        
        self.d_err = 0.0
        self.phi_err = 0.0

        self.speed_limit = _MED_SPD

        self.did_see_sign = False
        self.dist_to_sign = 0.0
        self.last_sign = 'NONE'

    def updateTurnOmega(self, tune_msg):
        msg = ""
        if tune_msg.direction.upper() == 'LEFT':
            _LEFT_OMEGA = tune_msg.omega
            _LEFT_DURATION = tune_msg.duration
            msg = """
                  _LEFT_OMEGA set    : {}
                  _LEFT_DURATION set : {}""".format( _LEFT_OMEGA, _LEFT_DURATION)
        elif tune_msg.direction.upper() == 'RIGHT':
            _RIGHT_OMEGA = tune_msg.omega
            _RIGHT_DURATION = tune_msg.duration
            msg = """
                  _RIGHT_OMEGA set : {}
                  _RIGHT_DURATION set : {}""".format( _RIGHT_OMEGA, _RIGHT_DURATION)
        else:
            msg = "Invalid message value : {}", tune_msg.direction
                    
        rospy.logerr(msg)
            
    def checkSign(self, sign_msg):
        sign = sign_msg.sign
        self.dist_to_sign = sign_msg.dist_to_sign

        if sign != 'NONE':
            self.did_see_sign = True
            self.last_sign = sign
    
    def arbitration(self, car_cmd_baseline):
        v = car_cmd_baseline.v
        o = car_cmd_baseline.omega
        # If there's no sign to handle or we saw 'GO', use LF
        if not self.did_see_sign or self.last_sign == 'GO':
            self.did_see_sign = False
            self.publish(v, o)
            return

        elif self.last_sign == 'STOP':
            if self.dist_to_sign <= _STOP_DISTANCE + 0.05 :
                self.publish(0.0, 0.0)
            else:
                self.publish(v, o)
            return

        elif self.last_sign == 'RIGHT' or self.last_sign == 'LEFT':
            if self.dist_to_sign <= 0.65 and not self.blocking:
                self.blocking = True                
                self.turn()
                self.last_sign = None
                self.did_see_sign = False
                self.blocking = False
                return
            else:
                self.publish(v, o)
            return
        
        elif self.last_sign == 'SLOW':
            if self.dist_to_sign <= 0.70:
                self.speed_limit = _LOW_SPD
                
        elif self.last_sign == 'FAST':
            if self.dist_to_sign <= 0.70:
                self.speed_limit = _MED_SPD

        self.did_see_sign = False
        self.publish(v, o)

    def checkLanePose(self, lanePose_msg):
        self.d_err = lanePose_msg.d
        self.phi_err = lanePose_msg.phi    

    def moveRobot(self, v, omega, duration_time, rate=100):
        # Generic function for moving the robot for a certain amount of time
        msg = """
        Velocity : {}
        Omega    : {}
        Duration : {}""".format(min(v, self.speed_limit), omega, duration_time)
        rospy.logerr(msg)
        start_time = rospy.get_time()
        rospy.logwarn("""
        Start Time : {}""".format(start_time))
        r = rospy.Rate(rate)
        while (rospy.get_time() - start_time) < duration_time:
            self.publish(v, omega)
            r.sleep()

        rospy.logwarn("""
        End time   : {}""".format(rospy.get_time()))

    def turn(self, vel=0.25):
        # Assume that the robot comes to a stop before executing a turn.
        # Use the phi value to center it
        if math.fabs(self.phi_err) > 0.2: 
            self.moveRobot(0, -2 * self.phi_err, 0.5)

        # Then, based on the sign, we turn
        # We set more or less aggressive omegas based on the position of the 
        # robot in the lane
        if self.last_sign == 'LEFT':
            self.moveRobot(vel, _LEFT_OMEGA, _LEFT_DURATION)
            #if math.fabs(self.d_err) <= 0.05:
             #   moveRobot(0.25, 2.2, 3)
            #elif self.d_err > 0.05:
             #   moveRobot(0.25, 1.9, 3)
            #elif self.d_err < -0.05:
             #   moveRobot(0.25, 2.5, 3)

        elif self.last_sign == 'RIGHT':
            if math.fabs(self.d_err) <= 0.05:
                self.moveRobot(0.25, -3.8, 1.5)
            elif self.d_err > 0.05:
                self.moveRobot(0.25, -3.9, 1.5)
            elif self.d_err < -0.05:
                self.moveRobot(0.25, -3.5, 1.5)

    def updateSelfState(self, selfState):
        self.curr_v = selfState.v
        self.curr_omega = selfState.omega

    def publish(self, v, o):
        output = Twist2DStamped()
        output.v = min(v, self.speed_limit)
        output.omega = o
        self.pub.publish(output)
        

        
if __name__=='__main__':
    try:
        rospy.init_node(_NODE_NAME, anonymous=True)
        Arbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TODO : Log Exception thrown")        
