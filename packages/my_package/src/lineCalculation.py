import numpy as np

def mainRed(edgepoligonRed, width):
    m, b = fit_line_through_points(edgepoligonRed)
    x1 = 0
    y1 = int(m * x1 + b)
    x2 = width - 1
    y2 = int(m * x2 + b)
    return x1, y1, x2, y2

def mainBlue(edgepoligonBlue, width,  hight, x1, y1, x2, y2):
        currentIndex = 0
        lowestX = float('inf')
        for index, currentEdgePoligon in enumerate(edgepoligonBlue):
            # find the side of the line
            side = getSideOfLine(x1, y1, x2, y2, currentEdgePoligon.x, currentEdgePoligon.y)
            if side == "left":
                continue
            for i in range(len(currentEdgePoligon.contours)):
                if currentEdgePoligon.contours[i][0][0] < lowestX:
                    lowestX = currentEdgePoligon.contours[i][0][0]
                    currentIndex = index
                currentEdgePoligon.contours[i][0][1] = currentEdgePoligon.contours[i][0][1] + hight // 2
    
        theLowestContour = edgepoligonBlue[currentIndex].contours
        coefficients = fit_lineByContours(theLowestContour[:, 0, :])
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
    m, b = coefficients
    return m, b

def find_orthogonal_line(edgePoligons, x1, y1, x2, y2):
    blue_edgePoligons = list(dict.fromkeys(edgePoligons))
    new_blue_edgePoligons = []
    # Find the center of the blue edge poligons
    x_coordinates = []
    y_coordinates = []
    highest_y = 0
    second_highest_y = 0
    highest_index = 0
    i = 0
    for edgePoligon in blue_edgePoligons:
        print("this is the edgepoligon: ", edgePoligon.x, edgePoligon.y)
        if getSideOfLine(x1, y1, x2, y2, edgePoligon.x, edgePoligon.y) == "left":
            #pop the edgepoligon from the list
            blue_edgePoligons.pop(i)
            i += 1
            continue
        if edgePoligon.y > highest_y:
            highest_index = i
            y_coordinates.append(edgePoligon.y)
            x_coordinates.append(edgePoligon.x)
            highest_y = edgePoligon.y
        i+=1
    #if less than 2 edgepoligons are left, return
    if len(blue_edgePoligons) < 2:
        return 0, 0
    #find the second highest y value
    new_blue_edgePoligons.append(blue_edgePoligons[highest_index])
    highest_index = 0
    i = 0
    for edgePoligon in blue_edgePoligons:
        if edgePoligon.y > second_highest_y and edgePoligon.y < highest_y:
            highest_index = i
            second_highest_y = edgePoligon.y
        i += 1

    new_blue_edgePoligons.append(blue_edgePoligons[highest_index])
    print(x_coordinates)
    print(y_coordinates)
    if len(x_coordinates) == 0:
        return 0, 0
    x_center = sum(x_coordinates) / len(x_coordinates)
    y_center = sum(y_coordinates) / len(y_coordinates)

    # Find the slope of the line through the center of the blue edge poligons
    m, b = fit_line_through_points(new_blue_edgePoligons)

    # Find the slope of the orthogonal line
    m_orthogonal = -1 / m

    # Find the y-intercept of the orthogonal line
    b_orthogonal = y_center - m_orthogonal * x_center

    return m_orthogonal, b_orthogonal

