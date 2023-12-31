#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import getYellow
import cv2
import numpy as np
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)




    def check_coordinates(self,image,edges):
        y_yellowstripes = 0
        x_yellowstripes = 0

        x_whiteline = 0
        y_whiteline = 0

        average_yellowstripes = 0
        average_whiteline = 0
        #divides edges into yellow and white
        for edge in edges:
            if edge.color[0] == 255:
                x_yellowstripes += edge.x
                y_yellowstripes += edge.y
                average_yellowstripes+= 1
            else:
                if edge.x < 150:
                    continue
                x_whiteline += edge.x
                y_whiteline += edge.y
                average_whiteline += 1
        if average_whiteline == 0:
            x_whiteline = 0
            y_whiteline = 0
        else:
            x_whiteline = x_whiteline/average_whiteline
            y_whiteline = y_whiteline/average_whiteline
        if average_yellowstripes == 0:
            x_yellowstripes = 0
            y_yellowstripes = 0
        else:
            x_yellowstripes = x_yellowstripes/average_yellowstripes
            y_yellowstripes = y_yellowstripes/average_yellowstripes

        _, current_middle, _ = image.shape
        current_middle = current_middle/2
        if (x_whiteline - current_middle) > 240:
            print("turn right")
        elif current_middle - x_yellowstripes > 250:
            print("turn left")
        else:
            print("no change in direction needed")

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        image, edges = getYellow.findFertures(image)
        self.check_coordinates(image, edges)
        #Display the image with the detected lines and midpoint
        cv2.imshow(self._window, image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()  # Close OpenCV windows
            rospy.signal_shutdown("User pressed 'q' key")

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()



