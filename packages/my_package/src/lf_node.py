#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.sub2 = rospy.Subscriber("laneInfo", String, self.lane_correction_callback)

    """
this is the msg one every new line is a new msg
blue right 0,-182,639,442
red left 0,546,639,-410
blue right 0,-182,639,442
red left 0,546,639,-410
blue right 0,-181,639,442
red left 0,546,639,-410
blue right 0,-182,639,442
red left 0,546,639,-410
blue right 0,-182,639,442
red left 0,546,639,-410
blue right 0,-183,639,443
red left 0,546,639,-410
    """
    def lane_correction_callback(self, msg):
        split_msg = msg.data.split()
        if split_msg[0] == "blue":
            x1 = int(split_msg[1])
            y1 = int(split_msg[2])
            x2 = int(split_msg[3])
            y2 = int(split_msg[4])





if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='lane_follower_node')
    # keep spinning
    rospy.spin()



