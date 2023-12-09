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
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        self.sub2 = rospy.Subscriber("laneInfo", String, self.lane_correction_callback)


    def lane_correction_callback(self, msg):
        print(msg.data)



if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()



