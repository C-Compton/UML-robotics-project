#!/usrbin/env python

import rospy

from duckietown_msgs.msg import LanePose, Twist2DStamped
from sign_reader.msg import SignInfo

_THRESHOLD_D = 0.0
_THRESHOLD_PHI = 0.0
_NODE_NAME = 'arbiter'

class Arbiter:
    
    def __init__(self):
        # Intercept the original message
        rospy.Subscriber('lane_controller_node/car_cmd', Twist2DStamped, queue_size=10)
        rospy.Subscriber('lane_controller_node/lane_pose', LanePose, self.checkLanePose)
        rospy.Subscriber('sign_reader_node/sign_info', SignInfo, self.checkSign)

        self.pub = rospy.Publisher('arbiter_controller_node/car_cmd', Twist2DStamped, queue_size=10)
        self.d_phi_oor = False
        self.did_see_sign = False

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