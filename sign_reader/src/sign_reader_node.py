#!/usr/bin/env python

import rospy
import math
from apriltags2_ros.msg import AprilTagDetectionArray
from sign_reader.msg import SignInfo

# We'll want to pull this out into a dict that is importable
signDict = { 
    20 : 'STOP',    # Stop sign
    74 : 'GO',      # Traffic light sign
    7  : 'LEFT',    # One way pointing left
    6  : 'RIGHT',   # One way pointing right
    96 : 'SLOW',    # Duck crossing
    2  : 'FAST',    # Yield ~ welcome to Massachusetts
    -1 : 'NONE'
}

class SignReader:
    def __init__(self):
        rospy.Subscriber(
            '/tunaboat/tag_detections', 
            AprilTagDetectionArray, 
            self.printer
        )

        self.pubSignInfo = rospy.Publisher('/sign_info', SignInfo, queue_size=10)

    def printer(self, aprilTagInfo):
        # If the length of the detections list is 0, we aren't seeing any signs
        if len(aprilTagInfo.detections):
            for detection in aprilTagInfo.detections:
                z = aprilTagInfo.detections[0].pose.pose.pose.position.z
                signId = aprilTagInfo.detections[0].id[0]
                self.pubSignInfo.publish(signDict[signId], z)
        else:
            self.pubSignInfo.publish(signDict[-1], -1)

if __name__ == '__main__':
    rospy.init_node('sign_reader_node', anonymous=True)
    SignReader()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

