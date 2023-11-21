import cam_en

#class egdePoligon:
#    def __init__(self, x, y, contour, side):
#        self.x = x
#        self.y = y
#        self.contour = contour
#        self.side = side
    

#print egdepoligonsj
#find how long the egdePoligon is from the center of middle of the image in the x-axis

#find the arage sum of the left and right egdePoligons
def findDistance(egdePoligons, centerOfimg):
    sumLeft = 0
    sumRight = 0
    for egdePoligon in egdePoligons:
        if egdePoligon.side == "left":
            sumLeft += egdePoligon.x
        else:
            sumRight += egdePoligon.x

    avgLeft = sumLeft / len(egdePoligons)
    avgRight = sumRight / len(egdePoligons)
    #print in pyhton3   
    print("avgLeft: " + str(avgLeft))
    print("avgRight: " + str(avgRight))



