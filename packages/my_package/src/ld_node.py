#!/usr/bin/env python3
import lineCalculation as lc
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import pickle 
import cv2
import numpy as np
from cv_bridge import CvBridge
from math import sqrt

class edgePoligon:
    def __init__(self, x, y, color, contours):
        self.x = x
        self.y = y
        self.color = color
        self.nameColor = self.findColor()
        self.contours = contours

    def findColor(self):
        if self.color == (0, 0, 255):
            return "red"
        elif self.color == (255, 0, 0):
            return "blue"


class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._redLine = []
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        self.currrentMsg = ""
        # create window
        self._window = "camera-reader"
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.pub = rospy.Publisher("lane_detection_correction", String, queue_size=1)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        image = self.findFertures(image)
        #create a http server with the image and and updateit

        cv2.imshow(self._window, image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()  #Close OpenCV windows
            rospy.signal_shutdown("User pressed 'q' key")
       #check_coordinates(image, edges)
        #Display the image with the detected lines and midpoint

    def findFertures(self, img):
        #create a list of egdepoligons that is empty
        egdepoligonYellow = []
        egdepoligonWhite = []
        egdepoligonRed = []
        #create a list of egdepoligons for each image
        height, width, _ = img.shape
        mask = np.zeros_like(img)
        mask[height // 2:, :] = 255
        #show the mask in a window with and height as the mask
        gray_mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        gray_roi = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_roi = cv2.bitwise_and(gray_roi, gray_roi, mask=gray_mask)
        #find the hlaf of the gray_roi 
        gray_roi = gray_roi[height // 2:, :]
        #show the hlaf high in imshow
        edges = cv2.Canny(gray_roi, 100, 200)
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area < 50:
                continue
            M = cv2.moments(contours[i])
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            y = y + height // 2
            b, g, r = img[y, x]
            #r g b as np array
            currebtPoint = np.array([[r, g, b]])
            #find the color of the point
            color = closest_centroid(currebtPoint)
            #if blue then images is white
            #add offset i contours
            if color == (0, 0, 255):
                egdepoligonWhite.append(edgePoligon(x, y, color, contours[i]))
            #if red then image is yellow
            elif color == (255, 0, 0):
                egdepoligonYellow.append(edgePoligon(x, y, color, contours[i]))
            elif color == (15,255,255):
                egdepoligonRed.append(edgePoligon(x, y, color, contours[i]))
            #if egdepoligonRed or egdepoligonWhite is empty return
        #show
        if len(egdepoligonYellow) != 0:
            x1, y1, x2, y2 = lc.mainYellow(egdepoligonYellow, width)
        else:
            x1 = -2
            y1 = -2
            x2 = -2
            y2 = -2

        if len(egdepoligonRed) > 1:
            rx1, ry1, rx2, ry2 = lc.mainRed(egdepoligonRed)
            cv2.line(img, (rx1, ry1), (rx2, ry2), (15, 255, 255), 3)
        else:
            rx1 = -1
            ry1 = -1
            rx2 = -1
            ry2 = -1

        lowestx = 100000
        currentInt = 0
        if len(egdepoligonWhite) > 0:
            for index, egdepoligon in enumerate(egdepoligonWhite):
                if lc.getSideOfLine(x1, y1, x2, y2, egdepoligon.x, egdepoligon.y) == "left" and len(egdepoligonYellow) != 0:
                    continue
                #draw the contour of the blue purple
                rx, ry, rw, rh = cv2.boundingRect(egdepoligon.contours)
                if rx < lowestx:
                    lowestx = rx
                    currentInt = index
            #dreaw the contour of the blue purple
            mx1, my1, mx2, my2 = lc.mainWhite(egdepoligonWhite[currentInt].contours, width)

            #offset the y in mx1, my1, mx2, my2
            my1 = my1 + height // 2
            my2 = my2 + height // 2

        else:
            #the width of the image
            mx1 = width
            my1 = width
            mx2 = width
            my2 = width

        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.line(img, (mx1, my1), (mx2, my2), (0, 0, 255), 3)
        #msg = x1, y1, x2, y2, mx1, my1, mx2, my2
        #turn the msg into a string
        msg = str(x1) + "," + str(y1) + "," + str(x2) + "," + str(y2) + "," + str(mx1) + "," + str(my1) + "," + str(mx2) + "," + str(my2) + "," + str(rx1) + "," + str(ry1) + "," + str(rx2) + "," + str(ry2)
        self.pub.publish(msg)
        return img



#point is the current color
def closest_centroid(point):
    secondList = [[191.62790698, 210.06976744,  73.09302326], [175.10843373, 192.15662651,  44.60240964], [176.20930233, 192.58139535,  11.76744186]]
    firstList = [168.85185185, 204.81481481, 184.66666667], [ 54.94444444,  78.16666667,  43.97222222], [109.10526316, 141.36842105, 117.84210526]
    thirdList = [199.30232558,  83.74418605,  73.37209302], [233.70588235, 103.76470588,  95.70588235], [237.7173913 ,  83.43478261,  75.95652174]
    first_centroid = firstList
    second_centroid = secondList
    third_centroid = thirdList
    currnt_shortest_distance = 100000
    currentColor = (0, 0, 0)
    for i in range(len(first_centroid)):
        #find the distance between the point and the first centroid
        distance1 = sqrt((point[0][0] - first_centroid[i][0])**2 + (point[0][1] - first_centroid[i][1])**2 + (point[0][2] - first_centroid[i][2])**2)
        #find the distance between the point and the second centroid
        distance2 = sqrt((point[0][0] - second_centroid[i][0])**2 + (point[0][1] - second_centroid[i][1])**2 + (point[0][2] - second_centroid[i][2])**2)
        distance3 = sqrt((point[0][0] - third_centroid[i][0])**2 + (point[0][1] - third_centroid[i][1])**2 + (point[0][2] - third_centroid[i][2])**2)
        if distance1 < currnt_shortest_distance:
            #white stripejj
            currentColor = (0, 0,255)
            currnt_shortest_distance = distance1
        if distance2 < currnt_shortest_distance:
            #yellow stripe
            currentColor = (255, 0,0)
            currnt_shortest_distance = distance2
        if distance3 < currnt_shortest_distance:
            #red stripe#yellow color
            currentColor = (15,255,255)
            currnt_shortest_distance = distance3
    return currentColor

if __name__ == "__main__":
    node = CameraReaderNode(node_name="camera_reader_node")
    rospy.spin()

