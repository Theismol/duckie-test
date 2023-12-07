from re import A
import cv2 
import duckietown_code_utils as dtu

import numpy as np

class ImageShow(dtu.Configurable):
    def __init__(self, configuration):
        self.image = np.empty(0)
        self.hsv = np.empty(0)

        param_names = [
                'hsv_white_low','hsv_white_high',]


        dtu.Configurable.__init__(self, param_names, configuration)


    def setImage(self, image):
        with dtu.timeit_clock('np.copy'):
            self.image = image
            self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)




