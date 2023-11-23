import cv2
import numpy as np
import os
import numpy as np
from math import sqrt
import ast
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler

#dataset one:
data = []
with open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/colors.txt', 'r') as file:
    for line in file:
        line_data = line.strip().split(' ')
        color = ast.literal_eval(' '.join(line_data[5:]))
        data.append(color)

data2 = [] 
with open('/code/catkin_ws/src/<duckie-test>/packages/my_package/src/other.txt', 'r') as file:
    for line in file:
        line_data = line.strip().split(' ')
        color = line_data[1:]
        #string to int
        color = [int(i) for i in color]
        data2.append(color)

X = np.array(data)
X2 = np.array(data2)

kmeans = KMeans(n_clusters=3, random_state=0).fit(X)
kmeans2 = KMeans(n_clusters=3, random_state=0).fit(X2)

centroids = kmeans.cluster_centers_
centroids2 = kmeans2.cluster_centers_

#find the closest centroid to the point
def closest_centroid(point):
    first_centroid = centroids
    second_centroid = centroids2

    #print first_centroid x, y, z
    print("first centroid:", first_centroid)
    #print second_centroid x, y, z
    print("second centroid:", second_centroid)
    #i have 2 centroids with 3 coordinates each
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
    print(someString)
    return someString


