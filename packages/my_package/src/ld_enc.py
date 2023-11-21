import cv2
import numpy as np


#image is cv2 image
# Function that can find all the images in the current path
def find_images(image):

    img = image
    height, width, _ = img.shape
    mask = np.zeros_like(img)
    mask[height // 2:, :] = 255
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
        cv2.circle(img, (x, y), 7, (255, 0, 0), -1)
        cv2.drawContours(img, contours[i:i+1], -1, (0, 0, 255), 2)
    return img


if __name__ == "__main__":
    find_images("C:\\Users\\alexa\\OneDrive\\Dokumenter\\AAU\\semsertre 3\\p3\\call")



