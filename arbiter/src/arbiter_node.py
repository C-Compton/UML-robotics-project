#!/usr/bin/env python

import rospy

from duckietown_msgs.msg import LanePose, Twist2DStamped
from sign_reader.msg import SignInfo


_NODE_NAME = 'arbiter_node'
_LOW_SPEED=0.1
_MED_SPEED=0.2
_HI_SPEED=0.5
_STOP_DISTANCE=0.22 # the width of one lane

from enum import Enum

class THRES_LEFT(Enum):
    D = 0.25
    PHI = 0.60

class THRES_RIGHT(Enum):
    D = 0.10
    PHI = 0.40
		
_NODE_NAME = 'arbiter'


class Arbiter:
    
    def __init__(self):
        # Intercept the original message
        rospy.Subscriber('lane_controller_node/car_cmd', Twist2DStamped, self.checkCarCmd)
        rospy.Subscriber('lane_filter_node/lane_pose', LanePose, self.checkLanePose)
        rospy.Subscriber('sign_reader_node/sign_info', SignInfo, self.checkSign)
        self.pub = rospy.Publisher('arbiter_controller_node/car_cmd', Twist2DStamped, queue_size=10)


        # D and Phi lane_pose out-of-range boolean
        # We expect the D and/or Phi values to exceed some pre-determined threshold. Once
        # the are back within bounds, we can assume that we've successfully navigated the
        # turn and the bot is detecting a good position in lane again.
        self.d_phi_oor = False
        self.did_see_sign = False
        self.d_values = []
        self.phi_values = []
        self.speed=_MED_SPEED # Twist2DStamped v
        self.heading=0 # Twist2DStamped omega


    def checkCarCmd(self,carCmd_baseline):
        if self.did_see_sign == False: # If there's no sign to handle, use LF
            self.speed = carCmd_baseline.v
            self.heading = carCmd_baseline.omega

    def checkLanePose(self, lanePose_msg):
        self.d_values.append(lanePose.d)
        self.phi_values.append(lanePose.phi)

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

        if sign is 'NONE':
            self.did_see_sign = False        
        else:
            self.did_see_sign = True
            self.last_sign = self.sign

            # Do stuff with sign message

        self.arbitration()


    def gentleStop(self):
        self.speed=0
        
        # Reduce distance to _STOP_DISTANCE.

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


    def arbitration(self):
        output = Twist2DStamped
        
        # Lots of logic here to determine 

        # Let's get chatty. At any given time, this node may be viewing:
        # SPEED: from 
            # gentleStop
            # turnLeft
            # turnRight
            # speedUp
            # slowDown 
            # Lane Follower (catch-all)

        # ORIENTATION: from 
            # turnLeft
            # turnRight
            # Lane Follower (catch-all)

        output.v=self.speed
        output.omega=self.heading


if __name__=='__main__':
    try:
        rospy.init_node(_NODE_NAME, anonymous=True)
        Arbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TODO : Log Exception thrown")        
