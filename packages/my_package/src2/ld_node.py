import cv2
import numpy as np
from math import sqrt
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
import pickle 
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class edgeFeature:
    def __init__(self, x, y, color ):
        self.x = x
        self.y = y
        self.color = color


class CameraReaderNode(DTROS):
    #def __init__(self, node_name):
       # self.right = False
       # self.left = False      # initialize the DTROS parent class
       # self.no_change = False
       # kmeans = pickle.load(open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/imgs/finalized_model1.sav', 'rb'))
       # kmeans1 = pickle.load(open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/imgs/finalized_model.sav', 'rb'))
       # self.centroids = kmeans.cluster_centers_
       # self.centroids1 = kmeans1.cluster_centers_
       # super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
       # # static parameters
       # self._vehicle_name = os.environ['VEHICLE_NAME']
       # self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
       # # bridge between OpenCV and ROS
       # self._bridge = CvBridge()
       # # create window
       # self._window = "camera-reader"
       # cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
       # self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
       # self.pub = rospy.Publisher("lane_detection_correction", String, queue_size=1)
    def __init__(self, path):
        kmeans = pickle.load(open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/imgs/finalized_model3.sav', 'rb'))
        kmeans1 = pickle.load(open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/imgs/finalized_model.sav', 'rb'))
        self.centroids = kmeans.cluster_centers_
        self.centroids1 = kmeans1.cluster_centers_
        images = [f for f in os.listdir(path) if f.startswith('img')]
        return images

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image, edges = self._bridge.compressed_imgmsg_to_cv2(msg)
        #findFertures(image)
        self.check_coordinates(image, edges)
        #Display the image with the detected lines and midpoint
        cv2.imshow(self._window, image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()  # Close OpenCV windows
            rospy.signal_shutdown("User pressed 'q' key")

    def findFertures(self, img):
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
        edgesFeatures = []
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area < 80:
                continue
            M = cv2.moments(contours[i])
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            #add the high of the mask to the y value
            y = y + (height // 2)
            b, g, r = img[y, x]
            #r g b as np array
            currebtPoint = np.array([[r, g, b]])
            #find the color of the point
            color = closest_centroid(currebtPoint)
            edgesFeatures.append(edgeFeature(x,y,color))
            #add the y offset to the i element in the contour list
            for j in range(len(contours[i])):
                contours[i][j][0][1] = contours[i][j][0][1] + (height // 2)
            cv2.drawContours(img, contours, i, color, 2)
        return img, edgesFeatures
    
    def closest_centroid(self, point):
        first_centroid = self.centroids
        second_centroid = self.centroids2
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
    node = CameraReaderNode("/imgs")
    imges = node.findFertures()
    for i in range(len(imges)):
        node.findFertures()

    # keep spinning
    rospy.spin()