import numpy as np

x1 = 0
y1 = 0
x2 = 0
y2 = 0


def mainRed(edgepoligonRed, width):
    m, b = fit_line_through_points(edgepoligonRed)
    x1 = 0
    y1 = int(m * x1 + b)
    x2 = width - 1
    y2 = int(m * x2 + b)
    return x1, y1, x2, y2

def mainBlue(edgepoligonBlue, width):
    m_orthogonal, b_orthogonal = find_orthogonal_line(x1, y1, x2, y2, edgepoligonBlue)
    mx1 = 0
    my1 = int(m_orthogonal * mx1 + b_orthogonal)
    mx2 = width - 1
    my2 = int(m_orthogonal * mx2 + b_orthogonal)
    return mx1, my1, mx2, my2



def getSideOfLine(x1, y1, x2, y2, x, y):
    # Calculate the distance from the point to the line
    distance = (y - y1) * (x2 - x1) - (x - x1) * (y2 - y1)
    # The point is on the left side if the distance is positive
    if distance > 0:
        return "right"
    # The point is on the right side if the distance is negative
    elif distance < 0:
        return "left"
    # The point is on the line if the distance is zero
    else:
        return "on the line"

def fit_line_through_points(edgePoligons):
    x_coordinates = []
    y_coordinates = []

    for edgePoligon in edgePoligons:
        x_coordinates.append(edgePoligon.x)
        y_coordinates.append(edgePoligon.y)
    # Fit a line (y = mx + b) through the points using linear regression
    print(x_coordinates)
    print(y_coordinates)
    coefficients = np.polyfit(x_coordinates, y_coordinates, 1)

    # Get the slope (m) and y-intercept (b)
    m, b = coefficients
    return m, b

def find_orthogonal_line(x1, y1, x2, y2, edgePoligons):
    blue_edgePoligons = list(dict.fromkeys(edgePoligons))
    # Find the center of the blue edge poligons
    x_coordinates = []
    y_coordinates = []
    for edgePoligon in blue_edgePoligons:
        if getSideOfLine(x1, y1, x2, y2, edgePoligon.x, edgePoligon.y) == "left":
            continue
        x_coordinates.append(edgePoligon.x)
        y_coordinates.append(edgePoligon.y)
    x_center = sum(x_coordinates) / len(x_coordinates)
    y_center = sum(y_coordinates) / len(y_coordinates)

    # Find the slope of the line through the center of the blue edge poligons
    m, b = fit_line_through_points(blue_edgePoligons)

    # Find the slope of the orthogonal line
    m_orthogonal = -1 / m

    # Find the y-intercept of the orthogonal line
    b_orthogonal = y_center - m_orthogonal * x_center

    return m_orthogonal, b_orthogonal

