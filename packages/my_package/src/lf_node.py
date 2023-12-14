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

def main(line_list, current_way):
    x1 = line_list[0]
    y1 = line_list[1]
    x2 = line_list[2]
    y2 = line_list[3]
    x3 = line_list[4]
    y3 = line_list[5]
    x4 = line_list[6]
    y4 = line_list[7]
    rx1 = line_list[8]
    ry1 = line_list[9]
    rx2 = line_list[10]
    ry2 = line_list[11]
    line_color = None
    # Calculate the slope of the lines
    try:
        yellow_slope = (y2 - y1) / (x2 - x1)
    except:
        forward = (0.0, 0.25)
        print("-----------------except----------------- in RED")
        return forward, line_color
    try:
        white_slope = (y4 - y3) / (x4 - x3)
    except:
        forward = (0.3,0.0)
        print("-----------------except----------------- in White")
        return forward, line_color
    if rx1 > 0:
        get_slides = (rx1 - x1) * (y2 - y1) - (ry1 - y1) * (x2 - x1)
        if get_slides > 0:
            print("-----------------RED IS IN THE LEFT-----------------")
        else:
            print("-----------------RED IS IN THE RIGHT-----------------")
            current_way, color = get_correction_red(rx1, ry1, rx2, ry2, current_way, yellow_slope, y1, y2)
            if color == "red":
                return current_way, color
            elif color == "yellow":
                return current_way, color
    x, y = find_interaction_point(x1, y1, x2, y2, x3, y3, x4, y4)
    # Calculate the distance from the center of the image to the intersection point in x axis
    distance = int(width / 2) - int(x)
    forward, line_color = get_correction_value(distance, white_slope, yellow_slope, current_way, y1, y2)
    return forward, line_color

    # Return the coordinates

def get_correction_red(rx1, ry1, rx2, ry2, current_way, yellow_slope, x1, x2):
    #print("-----RED-----", "rx1:", rx1, "ry1:", ry1, "rx2:", rx2, "ry2:", ry2, "yellowSlop:", yellowSlop, "x1:", x1, "x2:", x2)
    # Calculate the slope of red line rx1, ry1, rx2, ry2
    lines_color = None
    try:
        red_slope = (ry2 - ry1) / (rx2 - rx1)
        print("-------slop of red line-------", red_slope)
        if red_slope > -1.5 and red_slope <-0.15:
            #print("----------RED SLOP----------")
            forward = (0.0, 0.0)
            return forward, lines_color
    except:
        print("-----------------except-----------------RED SLOP")
        return current_way, lines_color
    if ry2 > 350:
        lines_color = "red"
    if rx1 > 300:
        if x1 > 750:
            lines_color = "yellow"
            forward = (0.2,0.0)
            #print("-----------YELLOW----------- is in the right", forward)
            return forward, lines_color
        forward = (0.3, 0.3)
        return forward, lines_color
    if rx1 < 150:#tjek den like igen 
        forward = (0.3, 0.3)
        return current_way, lines_color
    return current_way, lines_color


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
def get_correction_value(distance, white_slope, yellow_slope, forward, y1, y2):
    line_color = None
    if white_slope < 0.8 and white_slope >-2.0:
        line_color = "white"
        #print("----WHITE---------",whiteSlop)  
        forward = (0.23, 0.35)
        return forward, line_color
    if distance < 30 and distance > -30:
        if yellow_slope < -8:
            forward = (0.23, 0.05)
            print("-----Straight-----","----But YELLOW---------", yellow_slope)
            return forward, line_color
        line_color = "both"
        print("straight")
        forward = (0.33, 0.33)
        return forward, line_color
    if distance > 30:
        print("------LEFT-----", forward, distance, white_slope)
        line_color = "both"
        if forward[0] <0.05:
            forward = (0.0, 0.25)
            print("------LEFT-----", forward, distance, white_slope)
            return forward, line_color
        forward = forward[0]- 0.07, forward[1] 
        return forward, line_color
    if distance < -30:
        print("-------RIGHT-----", forward, distance)
        line_color = "both"
        if forward[0]<0.05:
            forward = (0.25, 0.0)
            return forward, line_color
        forward = forward[0] , forward[1] - 0.07
        print("-------IN RIGHT-----", forward, distance)
        return forward, line_color
    if forward[0] == 0.2:
        forward = (0.33, 0.33)
        return forward, line_color
    return forward, line_color

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
