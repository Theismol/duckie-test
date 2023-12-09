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
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler

class edgePoligon:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color
        self.nameColor = self.findColor()

    def findColor(self):
        if self.color == (0, 0, 255):
            return "red"
        elif self.color == (255, 0, 0):
            return "blue"


class CameraReaderNode(DTROS):
    def __init__(self, node_name):
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
        self.pub = rospy.Publisher("lane_detection_correction", String, queue_size=1)


    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        image, edges = self.findFertures(image)
        #check_coordinates(image, edges)
        #Display the image with the detected lines and midpoint
        cv2.imshow(self._window, image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()  # Close OpenCV windows
            rospy.signal_shutdown("User pressed 'q' key")


    def findFertures(self, img):
        egdepoligonRed = []
        egdepoligonBlue = []
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
            #rgb
            if color == (0, 0, 255):
                egdepoligonRed.append(edgePoligon(x, y, color))
            elif color == (255, 0, 0):
                egdepoligonBlue.append(edgePoligon(x, y, color))
            #draw the circle
            cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
            #show   
            x1, y1, x2, y2 = lc.mainRed(egdepoligonRed, width)
            mx1, my1, mx2, my2 = lc.mainBlue(egdepoligonBlue, width)
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.line(img, (mx1, my1), (mx2, my2), (0, 0, 255), 3)
        return img
    
kmeans = pickle.load(open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/imgs/finalized_model2.sav', 'rb'))
kmeans1 = pickle.load(open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/imgs/finalized_model.sav', 'rb'))
centroids = kmeans.cluster_centers_
centroids2 = kmeans1.cluster_centers_

def closest_centroid(point):
    first_centroid = centroids
    second_centroid = centroids2
    currnt_shortest_distance = 100000
    someString = (255, 0, 0)
    for i in range(len(first_centroid)):
        #find the distance between the point and the first centroid
        distance1 = sqrt((point[0][0] - first_centroid[i][0])**2 + (point[0][1] - first_centroid[i][1])**2 + (point[0][2] - first_centroid[i][2])**2)
        #find the distance between the point and the second centroid
        distance2 = sqrt((point[0][0] - second_centroid[i][0])**2 + (point[0][1] - second_centroid[i][1])**2 + (point[0][2] - second_centroid[i][2])**2)
        if distance1 < currnt_shortest_distance:
            #white stripe
            someString = (0, 0,255)
            currnt_shortest_distance = distance1
        if distance2 < currnt_shortest_distance:
            #yellow stripe
            someString = (255, 0,0)
            currnt_shortest_distance = distance2
    return someString

if __name__ == "__main__":
    node = CameraReaderNode(node_name="camera_reader_node")
    rospy.spin()



