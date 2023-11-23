import cv2
import numpy as np
import os
import findColor



class egdePoligon:
    def __init__(self, x, y, contour, color):
        self.x = x
        self.y = y
        self.contour = contour
        self.color = color

def findFertures(img):
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
        #add the high of the mask to the y value
        y = y + (height // 2)
        b, g, r = img[y, x]
        #r g b as np array
        currebtPoint = np.array([[r, g, b]])
        #find the color of the point
        color = findColor.closest_centroid(currebtPoint)
        #add the y offset to the i element in the contour list
        for j in range(len(contours[i])):
            contours[i][j][0][1] = contours[i][j][0][1] + (height // 2)
        cv2.drawContours(img, contours, i, color, 2)
    return img



