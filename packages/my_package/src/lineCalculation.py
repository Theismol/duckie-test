import numpy as np
import warnings
warnings.simplefilter(action='ignore', category=np.RankWarning)


def main_yellow(edge_polygon, width):
    coefficients = fit_line_through_points(edge_polygon)
    if coefficients[0] > 3: # if the slope is too high, set it to a lower value
        #[ -1.1754386  431.94736842]
        coefficients[0] = -1.1754386
        coefficients[1] = 0.94736842
    x1 = 0
    y1 = int(coefficients[0] * x1 + coefficients[1])
    x2 = width - 1
    y2 = int(coefficients[0] * x2 + coefficients[1])
    #get the global variables
    return x1, y1, x2, y2


def main_red(edge_polygon):
    highest_x = 0
    lowest_x = 100000
    y1 = 0
    y2 = 0
    for edge in edge_polygon:
        if edge.x < 50:
            continue
        if edge.x < lowest_x:
            lowest_x = edge.x
            y1 = edge.y
        if edge.x > highest_x:
            highest_x = edge.x
            y2 = edge.y

    return highest_x, y2, lowest_x, y1


def main_white(countour, width):
    coefficients = fit_line_by_contours(countour[:, 0, :])
    x1 = 0
    y1 = int(coefficients[0] * x1 + coefficients[1])
    x2 = width - 1
    y2 = int(coefficients[0] * x2 + coefficients[1])
    return x1, y1, x2, y2

#create a unktion that see if x and y is on the left or right side of a line
def get_side_of_line(x1, y1, x2, y2, x, y):
    if (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1) > 0:
        return "right"
    else:
        return "left"

def fit_line_by_contours(points):
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

