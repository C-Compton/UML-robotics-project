#!/usr/bin/env python

import rospy

from duckietown_msgs.msg import LanePose, Twist2DStamped
from sign_reader.msg import SignInfo
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
        rospy.Subscriber('lane_controller_node/car_cmd', Twist2DStamped, queue_size=10)
        rospy.Subscriber('lane_controller_node/lane_pose', LanePose, self.checkLanePose)
        rospy.Subscriber('sign_reader_node/sign_info', SignInfo, self.checkSign)

        self.pub = rospy.Publisher('arbiter_controller_node/car_cmd', Twist2DStamped, queue_size=10)

        # D and Phi lane_pose out-of-range boolean
        # We expect the D and/or Phi values to exceed some pre-determined threshold. Once
        # the are back within bounds, we can assume that we've successfully navigated the
        # turn and the bot is detecting a good position in lane again.
        self.d_phi_oor = False
        self.did_see_sign = False
        self.d_values = 0.0
        self.phi_values = 0.0

    def checkLanePose(self, lanePose_msg):
        self.d_values.append( abs(lanePose.d))
        self.phi_values.append(abs(lanePose.phi))

        if self.d_values.len() >= 20 and self.phi_values.len() >= 20:
            self.d_values.pop()
            self.phi_values.pop()

        d_error = sum(self.d_values) / float(len(self.d_values))
        phi_error = sum(self.phi_values) / float(len(self.phi_values))

        self.threshold
        if self.last_sign is 'LEFT':
            self.threshold = THRES_LEFT
        elif self.last_sign is 'RIGHT':
            self.threshold = THRES_RIGHT
        else:
            threshold = None
                    

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

    def arbitration(self):
        output = Twist2DStamped

        # Lots of logic here to determine 

        self.pub.publish(output)
        

if __name__=='__main__':
    try:
        rospy.init_node(_NODE_NAME, anonymous=True)
        Arbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TODO : Log Exception thrown")        
