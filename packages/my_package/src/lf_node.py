#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
#lane correction and following:

width = 640
height = 480

def main(x1, y1, x2, y2, x3, y3, x4, y4):
    # Calculate the slope of the lines
    redSlope = (y2 - y1) / (x2 - x1)
    blueSlope = (y4 - y3) / (x4 - x3)

    x, y = find_interaction_point(x1, y1, x2, y2, x3, y3, x4, y4)
    # Calculate the distance from the center of the image to the intersection point in x axis
    distance = int(width / 2) - int(x)
    print("this is the lfffff!:", x, y)
    correctionValue = getCorrectionValue(distance, blueSlope, redSlope)

    # Return the coordinates


def find_interaction_point(x1, y1, x2, y2, x3, y3, x4, y4):
    # Calculate the slope of the lines
    m1 = (y2 - y1) / (x2 - x1)
    m2 = (y4 - y3) / (x4 - x3)
    #print the slope of the lines
    # Calculate the y-intercepts
    b1 = y1 - m1 * x1
    b2 = y3 - m2 * x3
    # Calculate the x and y coordinates of the intersection to ints
    x = int((b2 - b1) / (m1 - m2))
    y = int(m1 * x + b1)

    # Return the coordinates
    return x, y
def getCorrectionValue(distance, blueSlope, redSlope):
    #is the distance positive or negative
    print(distance)
    if blueSlope < 1:
        print("tis turning left")
        return
    elif distance < 30 and distance > -30: 
        print("straight")
    elif distance > 30:
        print("right")
    elif distance < -30:
        print("left")
