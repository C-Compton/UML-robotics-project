#!/usr/bin/env python

import rospy

from duckietown_msgs.msg import LanePose, Twist2DStamped
from sign_reader.msg import SignInfo

_THRESHOLD_D = 0.0
_THRESHOLD_PHI = 0.0
_NODE_NAME = 'arbiter_node'
_LOW_SPEED=0.1
_MED_SPEED=0.2
_HI_SPEED=0.5
_STOP_DISTANCE=0.22 # the width of one lane

class Arbiter:
    
    def __init__(self):
        # Intercept the original message
        rospy.Subscriber('lane_controller_node/car_cmd', Twist2DStamped, self.checkCarCmd)
        rospy.Subscriber('lane_controller_node/lane_pose', LanePose, self.checkLanePose)
        rospy.Subscriber('sign_reader_node/sign_info', SignInfo, self.checkSign)

        self.pub = rospy.Publisher('arbiter_controller_node/car_cmd', Twist2DStamped, queue_size=10)
        self.d_phi_oor = False
        self.did_see_sign = False
        self.speed=_MED_SPEED # Twist2DStamped d
        self.heading=0 # Twist2DStamped omega

    def checkCarCmd(self,carCmd_baseline):
        if self.did_see_sign == False: # If there's no sign to handle, use LF
            self.speed=carCmd_baseline.d
            self.heading=carCmd_baseline.omega

    def checkLanePose(self, lanePose_msg):
        d_error = abs(lanePose.d)
        phi_error = abs(lanePose.phi)
        if d_error > _THRESHOLD_D or phi_error > _THRESHOLD_PHI:
            self.d_phi_oor = True
        else:
            self.d_phi_oor = False
    
    def checkSign(self, sign_msg):
        sign = sign_msg.sign
        if sign is 'NONE':
            self.did_see_sign = False        
        else:
            self.did_see_sign = True
            # Do stuff with sign message

        self.arbitration()

    def gentleStop(self,sign_msg):
        pass
        # Reduce distance to _STOP_DISTANCE.

    def slowDown(self,sign_msg):
        self.speed=_LOW_SPEED

    def speedUp(self,sign_msg):
        self.speed=_HI_SPEED

    def turnLeft(self,sign_msg):
        pass
        # Turn code

    def turnRight(self,sign_msg):
        pass        
        # Turn code




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


        self.pub.publish(output)
        

if __name__=='__main__':
    try:
        rospy.init_node(_NODE_NAME, anonymous=True)
        Arbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TODO : Log Exception thrown")        
