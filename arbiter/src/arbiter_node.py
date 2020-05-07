#!/usr/bin/env python

import rospy
import math

from duckietown_msgs.msg import LanePose, Twist2DStamped
from sign_reader.msg import SignInfo
from utilities.utils import PidController


_NODE_NAME = 'arbiter_node'
_LOW_SPEED=0.1
_MED_SPEED=0.2
_HI_SPEED=0.5
_STOP_DISTANCE=0.6 # the width of one lane
_SPEED_THRESHOLD=0.01

from enum import Enum

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


        # D and Phi lane_pose out-of-range boolean
        # We expect the D and/or Phi values to exceed some pre-determined threshold. Once
        # the are back within bounds, we can assume that we've successfully navigated the
        # turn and the bot is detecting a good position in lane again.
        self.blocking = False
        self.d_phi_oor = False
        self.did_see_sign = False
        self.d_values = []
        self.phi_values = []
        self.speed=_MED_SPEED # Twist2DStamped v
        self.heading=0 # Twist2DStamped omega
        self.vMultiplier = 1
        self.stopCtrl = PidController(0.5, 0, 0, 0, 0)
        self.did_see_sign = False

    def updateSelfState(self, selfState):
        self.curr_v = selfState.v
        self.curr_omega = selfState.omega

    def checkCarCmd(self,carCmd_baseline):
        if self.did_see_sign == False: # If there's no sign to handle, use LF
            self.speed = carCmd_baseline.v
            self.heading = carCmd_baseline.omega
        self.arbitration()

    def checkLanePose(self, lanePose_msg):
        self.d_values.append(lanePose_msg.d)
        self.phi_values.append(lanePose_msg.phi)

        while len(self.d_values) >= 20 and len(self.phi_values) >= 20:
            # TODO Probably not the most efficient way to do this but it works for now
            self.d_values.pop(0)
            self.phi_values.pop(0)

        d_error = sum(self.d_values) / float(len(self.d_values))
        phi_error = sum(self.phi_values) / float(len(self.phi_values))

        if self.last_sign is 'LEFT':
            self.threshold = THRES_LEFT
        elif self.last_sign is 'RIGHT':
            self.threshold = THRES_RIGHT
        else:
            self.threshold = None
                    
        if self.threshold is not None:
            if d_error > self.threshold.D or phi_error > self.threshold.PHI:
                self.d_phi_oor = True
            else:
                self.d_phi_oor = False
            # We need to clear these after a turn, not every pass.
            # self.d_values.clear()
            # self.phi_values.clear()
    
    def checkSign(self, sign_msg):
        self.sign = sign_msg.sign
        self.dist_to_sign = sign_msg.dist_to_sign

        if sign is 'NONE':
            self.did_see_sign = False        
        else:
            self.did_see_sign = True
            self.last_sign = self.sign

        # Should this be here or in chackCarCmd? If we call it here, Jooseppi
        # feels like it might miss wheel command messages
        # self.arbitration()

    def gentleStop(self):
        output = Twist2DStamped()
        previous_time = time.time()
        integ = 0
        error_dist = self.dist_to_sign - _STOP_DISTANCE # calculate error

        while(self.speed > _SPEED_THRESHOLD):
            current_time = time.time()
            ctrl_output = self.stopCtrl.pid(previous_time, current_time, error_dist)
            previous_time = current_time

            self.speed = ctrl_output
            output.v = self.speed
            output.omega = self.heading

            self.pub.publish(output)

        output.v = 0
        output.omega = 0
        self.pub.publish(output)

    def slowDown(self):
        self.speed=_LOW_SPEED

    def speedUp(self):
        self.speed=_HI_SPEED

    def moveRobot(self, v, omega, duration_time, rate=100):
        # Generic function for moving the robot for a certain amount of time
        output = Twist2DStamped()
        start_time = rospy.get_time()  
        while (rospy.get_time() - start_time) < duration_time:
            output.v = v
            output.omega = omega
            rate = rospy.Rate(rate)
            self.pub.publish(output)
            rate.sleep()

        output.v = 0
        output.omega = 0
        self.pub.publish(output)

    def turn(self):
        # Assume that the robot comes to a stop before executing a turn.
        # Use the phi value to center it
        if self.phi_values[-1] > 0.2 or self.phi_values[-1] < -0.2:
            self.moveRobot(0, -2 * self.phi_values[-1], 0.5)

        # Then, based on the sign, we turn
        # We set more or less aggressive omegas based on the position of the 
        # robot in the lane
        if self.last_sign == 'LEFT':
            if math.fabs(self.d_values[-1]) <= 0.05:
                moveRobot(0.25, 2.2, 3)
            elif self.d_values[-1] > 0.05:
                moveRobot(0.25, 1.9, 3)
            elif self.d_values[-1] < -0.05:
                moveRobot(0.25, 2.5, 3)

        elif self.last_sign == 'RIGHT':
            if math.fabs(self.d_values[-1]) <= 0.05:
                moveRobot(0.25, -3.8, 1.5)
            elif self.d_values[-1] > 0.05:
                moveRobot(0.25, -3.9, 1.5)
            elif self.d_values[-1] < -0.05:
                moveRobot(0.25, -3.5, 1.5)

        # Comment from Saba:
        # Go straight for 1 sec just to make sure we are in the
        # secound street.(it can be omited because if we set a
        # controler we do n$

    def go(self):
        output = Twist2DStamped()
        output.v = self.speed * self.vMultiplier
        output.omega = self.heading
        self.pub.publish(output)

    def arbitration(self):
        if not self.blocking:
            if self.sign_seen == 'STOP':
                self.blocking = True
                self.gentleStop()
                self.blocking = False

            elif self.sign_seen == 'LEFT':
                self.blocking = True
                self.gentleStop()
                self.turn()
                self.blocking = False

            elif self.sign_seen == 'RIGHT':
                self.blocking = True
                self.gentleStop()
                self.turn()
                self.blocking = False

            elif self.sign_seen == 'SLOW':
                self.vMultiplier = 0.5
                self.go()

            elif self.sign_seen == 'FAST':
                self.vMultiplier = 1
                self.go()

            else: # This also covers GO, FAST, and SLOW
                self.go()


if __name__=='__main__':
    try:
        rospy.init_node(_NODE_NAME, anonymous=True)
        Arbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TODO : Log Exception thrown")        
