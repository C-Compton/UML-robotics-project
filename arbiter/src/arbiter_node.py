#!/usr/bin/env python

import rospy

import math
import threading

from duckietown_msgs.msg import LanePose, Twist2DStamped, BoolStamped
from arbiter.msg import OmegaTune, SpeedTune
from sign_reader.msg import SignInfo
from utilities.utils import PidController
from enum import Enum

# Global constants
_NODE_NAME = 'arbiter_node'

# The approximate length of a Duckietown mat.
# The signs were to be placed on opposite sides of
# intersenctions to allow for adequate timing to see
# the sign if coming around a bend.
_STOP_DISTANCE=0.6 

class Speed:
    def __init__(self):
        self.speed_limit = "med"

        self.caps = {"low":0.1, "med":0.2, "hi":0.5}

    def limit(self):
        return self.caps.get(self.speed_limit, 0.2)

    def set_caps(self, low, med, hi):
        self.caps["low"] = low
        self.caps["med"] = med
        self.caps["hi"] = hi
    
class Arbiter:
    
    def __init__(self):
        # Intercept the original message
        rospy.Subscriber('lane_controller_node/car_cmd', Twist2DStamped, self.arbitration)
        rospy.Subscriber('lane_filter_node/lane_pose', LanePose, self.checkLanePose)
        rospy.Subscriber('sign_reader_node/sign_info', SignInfo, self.checkSign)
        rospy.Subscriber('arbiter_node/new_car_cmd', Twist2DStamped, self.updateSelfState)
        rospy.Subscriber('omega_tune/tune', OmegaTune, self.tuneTurnOmega)
        rospy.Subscriber('speed_tune/tune', SpeedTune, self.tuneSpeedLimits)
        self.pub = rospy.Publisher('arbiter_node/new_car_cmd', Twist2DStamped, queue_size=10)

        # Non-blocking pseudo-mutex
        # Used only when turning
        self.blocking = False

        # Originally used to dynamically calculate
        # v, omega of turns
        self.d_err = 0.0
        self.phi_err = 0.0

        # Left and Right turning constants
        self.L_OMEGA = 2.0
        self.L_DURATION = 1.5
        self.R_OMEGA = 4.0
        self.R_DURATION = 1.5

        # States to keep track of signs spotted
        self.did_see_sign = False
        self.dist_to_sign = 0.0
        self.last_sign = 'NONE'

        # instance of speed class for keeping track of
        # current speed limit and updating limits when
        # tuning
        self.speed = Speed()


    
    def checkSign(self, sign_msg):
        sign = sign_msg.sign
        self.dist_to_sign = sign_msg.dist_to_sign

        if sign != 'NONE':
            self.did_see_sign = True
            self.last_sign = sign

    # The meat and potatoes
    def arbitration(self, car_cmd_baseline):
        # First extract the base line values form the lane_controller_node
        v = car_cmd_baseline.v
        o = car_cmd_baseline.omega
        
        # If there's no sign to handle or we saw 'GO', use LF
        if not self.did_see_sign or self.last_sign == 'GO':
            self.did_see_sign = False
            self.publish(v, o)
            return

        elif self.last_sign == 'STOP':
            # Only execute the stop if we're close enough, with an added buffer for
            # reaction delay.
            if self.dist_to_sign <= _STOP_DISTANCE + 0.05 :
                self.publish(0.0, 0.0)

            # Otherwise, continue executing the LF values
            else:
                self.publish(v, o)
            return

        elif self.last_sign == 'RIGHT' or self.last_sign == 'LEFT':
            if self.dist_to_sign <= 0.65 and not self.blocking:
                # self.turn() rus in an open loop right now,
                # so block with a boolean to prevent
                # multiple open loops queueing up
                self.blocking = True                
                self.turn()
                self.last_sign = None
                self.did_see_sign = False
                self.blocking = False
                return
            # If we're not blocking, publish LF values
            # if we are blocking, then we're in the turn() method
            elif not self.blocking:
                self.publish(v, o)
            return
        
        elif self.last_sign == 'SLOW':
            # Only execute when within a set distance.
            if self.dist_to_sign <= 0.70:
                self.speed.speed_limit = "low"
                self.did_see_sign = False
                self.publish(v, o)
                return
            else:
                self.publish(v, o)
                return
                
        elif self.last_sign == 'FAST':
            if self.dist_to_sign <= 0.70:
                self.speed.speed_limit = "med" # Opted for low and med speeds only for time being
                self.did_see_sign = False
                self.publish(v, o)
                return
            else:
                self.publish(v, o)
                return

    def checkLanePose(self, lanePose_msg):
        self.d_err = lanePose_msg.d
        self.phi_err = lanePose_msg.phi    

    def moveRobot(self, v, omega, duration_time, rate=100):
        # Generic function for moving the robot for a certain amount of time
        self.logMoveRobot(min(v, self.speed.limit()), omega, duration_time)
        start_time = rospy.get_time()
        rospy.logdebug("""
        Start Time : {}""".format(start_time))
        r = rospy.Rate(rate)
        while (rospy.get_time() - start_time) < duration_time:
            self.publish(v, omega)
            r.sleep()

        rospy.logdebug("""
        End time   : {}""".format(rospy.get_time()))

        # Allow the lane_filter_node to
        # re-establish a lane (hopefully)
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) <= 0.66: # Make tunable?
              self.publish(v, 0.0)
              
    def turn(self, vel=0.25):
        # Assume that the robot comes to a stop before executing a turn.
        # Use the phi value to center it
        if math.fabs(self.phi_err) > 0.2: 
            self.moveRobot(0, -2 * self.phi_err, 0.5)

        if self.last_sign == 'LEFT':
            self.moveRobot(vel, self.L_OMEGA, self.L_DURATION)

        elif self.last_sign == 'RIGHT':
            self.moveRobot(vel, self.R_OMEGA, self.R_DURATION)

    # Orignally used the updated state values with the stop method's
    # PID controller, as what this node published was the ground
    # truth of the current v, omega values published
    def updateSelfState(self, selfState):
        self.curr_v = selfState.v
        self.curr_omega = selfState.omega

    def publish(self, v, o):
        output = Twist2DStamped()
        output.v = min(v, self.speed.limit()) # ensure we don't go above our set speed limit
        output.omega = o
        self.pub.publish(output)


    #
    # UTILITY METHODS
    #

    def logMoveRobot(self, v, omega, duration_time):
        msg = """
        Velocity : {}
        Omega    : {}
        Duration : {}""".format(v, omega, duration_time)
        rospy.logdebug(msg)
        

    def tuneSpeedLimits(self, tune_msg):
        low = tune_msg.low
        med = tune_msg.med
        hi = tune_msg.hi
        if low <= med and med <= hi :
            self.speed.set_caps(low, med, hi)
            caps = self.speed.caps
            msg = """
            LOW SPD : {}
            MED SPD : {}
            HI SPD  : {}
        
            CUR LMT : {}""".format(caps["low"], caps["med"], caps["hi"], self.speed.limit())
        else:
            msg = """
            Invalid message values:
                LOW : {}
                MED : {}
                HI  : {}
            Must be LOW <= MED <= HI

            CUR LMT : {}""".format(low, med, hi, self.speed.limit())

        rospy.logdebug(msg)

        
    def tuneTurnOmega(self, tune_msg):
        msg = ""
        if tune_msg.direction.upper() == 'LEFT':
            self.L_OMEGA = tune_msg.omega
            self.L_DURATION = tune_msg.duration
            msg = """
                  L_OMEGA    : {}
                  L_DURATION : {}""".format( self.L_OMEGA, self.L_DURATION)
        elif tune_msg.direction.upper() == 'RIGHT':
            self.R_OMEGA = tune_msg.omega
            self.R_DURATION = tune_msg.duration
            msg = """
                  R_OMEGA    : {}
                  R_DURATION : {}""".format( self.R_OMEGA, self.R_DURATION)
        else:
            msg = "Invalid message value : {}", tune_msg.direction
                    
        rospy.logdebug(msg)

        
if __name__=='__main__':
    try:
        rospy.init_node(_NODE_NAME, anonymous=True)
        Arbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException")        
