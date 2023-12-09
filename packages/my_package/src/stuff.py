def check_coordinates(self,image,edges):
    y_yellowstripes = 0
    x_yellowstripes = 0

    x_whiteline = 0
    y_whiteline = 0

    average_yellowstripes = 0
    average_whiteline = 0
    #divides edges into yellow and white
    for edge in edges:
        if edge.color[0] == 255:
            x_yellowstripes += edge.x
            y_yellowstripes += edge.y
            average_yellowstripes+= 1
        else:
            if edge.x < 150:
                continue
            x_whiteline += edge.x
            y_whiteline += edge.y
            average_whiteline += 1
    if average_whiteline == 0:
        x_whiteline = 0
        y_whiteline = 0
    else:
        x_whiteline = x_whiteline/average_whiteline
        y_whiteline = y_whiteline/average_whiteline
    if average_yellowstripes == 0:
        x_yellowstripes = 0
        y_yellowstripes = 0
    else:
        x_yellowstripes = x_yellowstripes/average_yellowstripes
        y_yellowstripes = y_yellowstripes/average_yellowstripes
    _, current_middle, _ = image.shape
    current_middle = current_middle/2
    if (x_whiteline - current_middle) > 230:
        print("turn right")
        self.right = True
        self.left = False
        self.no_change = False
        message = "right"
    elif current_middle - x_yellowstripes > 250:
        print("turn left")
        self.left = True
        self.right = False
        self.no_change = False
        message = "left"
    else:
        print("no change in direction needed")
        self.no_change = True
        self.right = False
        self.left = False
        message = "no_change"

    self.pub.publish(message)
