#create a program that can crop an image using cv2

import cv2
import numpy as np
import glob

#create a funktion that gets alle the pngs from a folder
def cropAll():
    somename = 0
    for img in glob.glob("*.jpg"):
        cropImg(img, somename)
        somename += 1
        #save the image


def cropImg(img, somename):
    img = cv2.imread(img)

    x1, y1 = 51, 113
    x2, y2 = 689, 593

    crop_img = img[y1:y2, x1:x2]
    #write the image to a file "img%d.png" % somename"
    cv2.imwrite("img%d.jpg" % somename, crop_img)

cropAll()
