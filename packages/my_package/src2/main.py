import threading
import time
from queue import Queue

import a_star as a_star
import lf_node as lf_node
import Wheel_node as Wheel_node
import ld_node as ld_node

directions = []


def generate_directions(start_node, goal_node):
    global path
    path = a_star.astar(start_node, goal_node, a_star.graph)[0]
    return a_star.get_directions(path)


def send_instructions():
    while len(directions) > 0:
        if len(directions) > 0:
            instruction = directions.pop(0)
            Wheel_node.run(instruction)
            time.sleep(1)
        


if __name__ == '__main__':
    start_node = (10, 6)
    goal_node = (10, 2)
    
    directions = generate_directions(start_node, goal_node)
    
    
    
    direction_thread = threading.Thread(target=send_instructions)
    
    direction_thread.start()
    direction_thread.join()
    
    
    

