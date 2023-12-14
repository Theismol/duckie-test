#!/usr/bin/env python3
import a_star as astar


def main():
    directions = astar.getRoad()
    print(directions)
    print("------A*------")
    splt = str(directions).split(",")
    print(splt)
    #remove curly braces
    list_of_directions = []
    for index, item in enumerate(splt):
        if index % 2 != 0:
            if item[6] == "f":
                continue
            list_of_directions.append(item[6])
    return list_of_directions

list_of_directions = main()
print(list_of_directions)

def get_directions():
    return list_of_directions[0]

def remove_first():
    list_of_directions.pop(0)
