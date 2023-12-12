import numpy as np
import warnings
warnings.simplefilter(action='ignore', category=np.RankWarning)


def mainYellow(edgepoligon, width):
    coeffic = fit_line_through_points(edgepoligon)
    if coeffic[0] > 3: # if the slope is too high, set it to a lower value
        #[ -1.1754386  431.94736842]
        coeffic[0] = -1.1754386
        coeffic[1] = 0.94736842
    x1 = 0
    y1 = int(coeffic[0] * x1 + coeffic[1])
    x2 = width - 1
    y2 = int(coeffic[0] * x2 + coeffic[1])
    #get the global variables
    return x1, y1, x2, y2


def mainRed(edgepoligon):
    highestX = 0
    lowestX = 100000
    y1 = 0
    y2 = 0
    for egde in edgepoligon:
        if egde.x < 50:
            continue
        if egde.x < lowestX:
            lowestX = egde.x
            y1 = egde.y
        if egde.x > highestX:
            highestX = egde.x
            y2 = egde.y

    return highestX, y2, lowestX, y1


def mainWhite(countour, width):
    coefficients = fit_lineByContours(countour[:, 0, :])
    x1 = 0
    y1 = int(coefficients[0] * x1 + coefficients[1])
    x2 = width - 1
    y2 = int(coefficients[0] * x2 + coefficients[1])
    return x1, y1, x2, y2

#create a unktion that see if x and y is on the left or right side of a line
def getSideOfLine(x1, y1, x2, y2, x, y):
    if (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1) > 0:
        return "right"
    else:
        return "left"

def fit_lineByContours(points):
    x_coordinates, y_coordinates = zip(*points)
    coefficients = np.polyfit(x_coordinates, y_coordinates, 1)
    return coefficients


def fit_line_through_points(edgePoligons):
    x_coordinates = []
    y_coordinates = []

    for edgePoligon in edgePoligons:
        x_coordinates.append(edgePoligon.x)
        y_coordinates.append(edgePoligon.y)
    # Fit a line (y = mx + b) through the points using linear regression
    coefficients = np.polyfit(x_coordinates, y_coordinates, 1)
    # Get the slope (m) and y-intercept (b)
    return coefficients

