import cv2
import numpy as np
import os
import wheel_cam 

class egdePoligon:
    def __init__(self, x, y, contour, side):
        self.x = x
        self.y = y
        self.contour = contour
        self.side = side

def test():
    print("test")
 # Function that can find all the images in the current path
def find_images(path):
    images = [f for f in os.listdir(path) if f.startswith('img')]
    i = 1
    for image in images:
        img = cv2.imread(os.path.join(path, image))
        #create a list of egdepoligons for each image
        egdepoligons = []
        height, width, _ = img.shape
        mask = np.zeros_like(img)
        mask[height // 2:, :] = 255
        #create a left side mask and a right side mask by using the bottom half of the image
        gray_mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY) 
        gray_roi = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_roi = cv2.bitwise_and(gray_roi, gray_roi, mask=gray_mask)
        edges = cv2.Canny(gray_roi, 100, 200)
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area < 50:
                continue
            M = cv2.moments(contours[i])
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            #find if x or y is in the left or right side of the image
            if x < width // 2:
                cv2.drawContours(img, contours[i:i+1], -1, (0, 224, 0), 2)
                egde = egdePoligon(x, y, contours[i], "left")
                egdepoligons.append(egde)
            else:
                cv2.drawContours(img, contours[i:i+1], -1, (0, 0, 255), 2)
                egde = egdePoligon(x, y, contours[i], "right")
                egdepoligons.append(egde)
            cv2.circle(img, (x, y), 7, (255, 0, 0), -1)
            cv2.imshow("img", img)
            #img = cv2.imread(os.path.join(path, image))
        if cv2.waitKey(0) & 0xFF == ord('w'):
            wheel_cam.findDistance(egdepoligons, width // 2)
        elif cv2.waitKey(0) & 0xFF == ord('q'):
            return egdepoligons


if __name__ == "__main__":
    find_images("C:\\Users\\alexa\\OneDrive\\Dokumenter\\AAU\\semsertre 3\\p3\\call")

