import threading
import time
from queue import Queue

import a_star as a_star
import lf_node as lf_node

directions = []
lane_queue = Queue()


def generate_directions():
    global path
    start_node = (7, 0)
    goal_node = (0, 9)
    path = a_star.astar(start_node, goal_node, a_star.graph)[0]
    return a_star.get_directions(path)


def lane_following_monitor():
    while True:
        if not lane_queue.empty():
            directions = lane_queue.get()
            print(directions)
        time.sleep(0.1)



if __name__ == '__main__':
    start_node = (10, 6)
    goal_node = (10, 2)
    
    directions = generate_directions()
    
    print(directions)
    
