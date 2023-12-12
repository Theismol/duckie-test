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

def main(lineList, currenWay):
    x1 = lineList[0]
    y1 = lineList[1]
    x2 = lineList[2]
    y2 = lineList[3]
    x3 = lineList[4]
    y3 = lineList[5]
    x4 = lineList[6]
    y4 = lineList[7]
    rx1 = lineList[8]
    ry1 = lineList[9]
    rx2 = lineList[10]
    ry2 = lineList[11]
    lineColor = None
    # Calculate the slope of the lines
    try:
        yellowSlop = (y2 - y1) / (x2 - x1)
        whiteSlop = (y4 - y3) / (x4 - x3)
        if rx1 > 0:
            currenWay, color = getCorrectionRed(rx1, ry1, rx2, ry2, currenWay, yellowSlop)
            if color == "red":
                return currenWay, color
        x, y = find_interaction_point(x1, y1, x2, y2, x3, y3, x4, y4)
        # Calculate the distance from the center of the image to the intersection point in x axis
        distance = int(width / 2) - int(x)
        forward, lineColor= getCorrectionValue(distance, whiteSlop, yellowSlop, currenWay)
    except:
        print("-----------------except-----------------")
        whiteSlop = (y4 - y3) / (x4 - x3)
        forward = (0.11, 0.3)
        forward = setException(whiteSlop)
    return forward, lineColor

    # Return the coordinates

def getCorrectionRed(rx2, ry1, rx1, ry2, currenWay, yellowSlop):
    print("-----RED-----", "rx1:", rx1, "ry1:", ry1, "rx2:", rx2, "ry2:", ry2, "yellowSlop:", yellowSlop)
    linesColor = None
    if ry2 > 350:
        print("-----RED----- STOP")
        linesColor = "red"
    if yellowSlop < -1:
        forward = (0.35, 0.3)
        if rx1 > 400:
            forward = (0.2, 0.0)
        return forward, linesColor


    return currenWay, linesColor


def setException(whiteSlope):
    print("----In Excation and WihteSlope:----", whiteSlope)
    if whiteSlope > 2:
        forward = (0.0, 0.2)
        print("-----IN IF---")
    else:
        print("-----IN ELSE---")
        forward = (0.0, 0.2)
    return forward

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
def getCorrectionValue(distance, whiteSlop, yellowSlop, forward):
    lineColor = None
    if whiteSlop < 0.8 and whiteSlop >-2.0:
        lineColor = "white"
        print("----WHITE---- --left turn---",whiteSlop)
        print("-----YELLOW-----",yellowSlop)
        forward = (0.23, 0.35)
        return forward, lineColor
    if distance < 30 and distance > -30:
        lineColor = "both"
        print("straight")
        forward = (0.33, 0.3)
        return forward, lineColor
    if distance > 30:
        print("------LEFT-----", forward, distance, whiteSlop)
        lineColor = "both"
        if forward[0] <0.05:
            forward = (0.0, 0.2)
            return forward, lineColor
        forward = forward[0]- 0.07, forward[1] 
        return forward, lineColor
    if distance < -30:
        print("-------RIGHT-----", forward, distance)
        lineColor = "both"
        if forward[0] == 0.23:
            forward = (0.25, 0.0)
            return forward, lineColor
        if forward[0]<0.05:
            forward = (0.15, 0.0)
            return forward, lineColor
        forward = forward[0] , forward[1] - 0.07
        print("-------IN RIGHT-----", forward, distance)
        return forward, lineColor
    return forward, lineColor

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
