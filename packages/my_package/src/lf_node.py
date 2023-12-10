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

def main(x1, y1, x2, y2, x3, y3, x4, y4, currenWay):
    print(currenWay, "this is wat the current way is")
    # Calculate the slope of the lines
    try:
        redSlope = (y2 - y1) / (x2 - x1)
        blueSlope = (y4 - y3) / (x4 - x3)
        x, y = find_interaction_point(x1, y1, x2, y2, x3, y3, x4, y4)
        # Calculate the distance from the center of the image to the intersection point in x axis
        distance = int(width / 2) - int(x)
        forward = getCorrectionValue(distance, blueSlope, redSlope, currenWay)
    except:
        print("-----------------except-----------------")
        forward = (0.2, 0.55)
    return forward

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
#FORWARD = (0.2,0.23)
def getCorrectionValue(distance, blueSlope, redSlope, forward):
    if distance < 30 and distance > -30:
        print("straight")
        forward = (0.3, 0.33)
        return forward
    if distance > 30:
        print("left turn", distance)
        forward = forward[0] -0.07, forward[1] 
        return forward
    if distance < -30:
        print("right turn", distance)
        forward = forward[0] , forward[1]- 0.07
        return forward
    return forward

    #if blueSlope < 1:
    #    print("Left turn", distance)
    #    forward = (0.35, 0.1)
    #    return forward
    #elif distance < 15 and distance > -15:
    #    forward = (0.33, 0.33)
    #    return forward
    #elif distance > 15:
    #    #multiply by the distance withe some factor so is a comma number
    #    distance = distance * 0.001
    #    forward = (0.3 + distance , 0.33)
    #    print("-----right ------the distance is: ", distance, "the forward is: ", forward)
    #    return forward
    #elif distance < -15:
    #    distance = distance * -1
    #    #multiply by the distance withe some factor so is a comma number
    #    distance = distance * 0.001
    #    forward = (0.3, 0.33 + distance)
    #    #print the distance and the forward with text
    #    print("------left -------the distance is: ", distance, "the forward is: ", forward)
    #    return forward
    #forward = (0.3, 0.33)

    #return forward


