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
    except:
        forward = (0.0, 0.25)
        print("-----------------except----------------- in RED")
        return forward, lineColor
    try:
        whiteSlop = (y4 - y3) / (x4 - x3)
    except:
        forward = (0.3,0.0)
        print("-----------------except----------------- in White")
        return forward, lineColor
    if rx1 > 0:
        getSides = (rx1 - x1) * (y2 - y1) - (ry1 - y1) * (x2 - x1)
        if getSides > 0:
            print("-----------------RED IS IN THE LEFT-----------------")
        else:
            print("-----------------RED IS IN THE RIGHT-----------------")
            currenWay, color = getCorrectionRed(rx1, ry1, rx2, ry2, currenWay, yellowSlop, y1, y2)
            if color == "red":
                return currenWay, color
            elif color == "yellow":
                return currenWay, color
    x, y = find_interaction_point(x1, y1, x2, y2, x3, y3, x4, y4)
    # Calculate the distance from the center of the image to the intersection point in x axis
    distance = int(width / 2) - int(x)
    forward, lineColor= getCorrectionValue(distance, whiteSlop, yellowSlop, currenWay, y1, y2)
    return forward, lineColor

    # Return the coordinates

def getCorrectionRed(rx1, ry1, rx2, ry2, currenWay, yellowSlop, x1, x2):
    #print("-----RED-----", "rx1:", rx1, "ry1:", ry1, "rx2:", rx2, "ry2:", ry2, "yellowSlop:", yellowSlop, "x1:", x1, "x2:", x2)
    # Calculate the slope of red line rx1, ry1, rx2, ry2
    linesColor = None
    try:
        redSlop = (ry2 - ry1) / (rx2 - rx1)
        print("-------slop of red line-------", redSlop)
        if redSlop > -1.5 and redSlop < 0.0:
            #print("----------RED SLOP----------")
            forward = (0.0, 0.0)
            return forward, linesColor
    except:
        print("-----------------except-----------------RED SLOP")
        return currenWay, linesColor
    if ry2 > 350:
         linesColor = "red"
    if rx1 > 300:
        if x1 > 750:
            linesColor = "yellow"
            forward = (0.2,0.0)
            #print("-----------YELLOW----------- is in the right", forward)
            return forward, linesColor
        forward = (0.3, 0.3)
        return forward, linesColor
    if rx1 < 150:#tjek den like igen 
        forward = (0.3, 0.3)
        return currenWay, linesColor
    return currenWay, linesColor


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
def getCorrectionValue(distance, whiteSlop, yellowSlop, forward, y1, y2):
    lineColor = None
    if whiteSlop < 0.8 and whiteSlop >-2.0:
        lineColor = "white"
        #print("----WHITE---------",whiteSlop)  
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
            forward = (0.0, 0.25)
            print("------LEFT-----", forward, distance, whiteSlop)
            return forward, lineColor
        forward = forward[0]- 0.07, forward[1] 
        return forward, lineColor
    if distance < -30:
        print("-------RIGHT-----", forward, distance)
        lineColor = "both"
        if forward[0]<0.05:
            forward = (0.25, 0.0)
            return forward, lineColor
        forward = forward[0] , forward[1] - 0.07
        print("-------IN RIGHT-----", forward, distance)
        return forward, lineColor
    if forward[0] == 0.2:
        forward = (0.33, 0.33)
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
