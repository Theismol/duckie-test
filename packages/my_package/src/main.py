#!/usr/bin/env python3
import a_star as astar


def main():
    directions = astar.getRoad()
    print(directions)
    print("------A*------")
    splt = str(directions).split(",")
    print(splt)
    #remove curly braces
    listOfDirections = []
    for index, item in enumerate(splt):
        if index % 2 != 0:
            if item[6] == "f":
                continue
            listOfDirections.append(item[6])
    return listOfDirections

listOfDirections = []
listOfDirections = main()
print(listOfDirections)

def getDirections():
    return listOfDirections[0]

def removeFirst():
    listOfDirections.pop(0)
