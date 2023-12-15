#!/usr/bin/env python3
import a_star as astar


def main():
    directions = astar.getRoad()
    print(directions)
    print("------A*------")
    directions.append("s")
    return directions

list_of_directions = main()

def get_directions():
    return list_of_directions[0]

def remove_first():
    list_of_directions.pop(0)

