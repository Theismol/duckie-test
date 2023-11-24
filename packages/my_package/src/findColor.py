import cv2
import numpy as np
import os
from math import sqrt
import ast
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
import pickle 
#dataset one: code/catkin_ws/src/<duckie-test>/packages/my_package/src/findiles.sva

kmeans = pickle.load(open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/imgs/finalized_model2.sav', 'rb'))
kmeans2 = pickle.load(open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/imgs/finalized_model.sav', 'rb'))

centroids = kmeans.cluster_centers_
centroids2 = kmeans2.cluster_centers_

#find the closest centroid to the point
def closest_centroid(point):
    first_centroid = centroids
    second_centroid = centroids2
    currnt_shortest_distance = 100000
    someString = (255, 0, 0 )
    for i in range(len(first_centroid)):
        #find the distance between the point and the first centroid
        distance1 = sqrt((point[0][0] - first_centroid[i][0])**2 + (point[0][1] - first_centroid[i][1])**2 + (point[0][2] - first_centroid[i][2])**2)
        #find the distance between the point and the second centroid
        distance2 = sqrt((point[0][0] - second_centroid[i][0])**2 + (point[0][1] - second_centroid[i][1])**2 + (point[0][2] - second_centroid[i][2])**2)
        if distance1 < currnt_shortest_distance:
            someString = (0, 0,255)
            currnt_shortest_distance = distance1
        if distance2 < currnt_shortest_distance:
            someString = (255, 0,0)
            currnt_shortest_distance = distance2
    return someString


