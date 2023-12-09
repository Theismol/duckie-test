#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
import os
import a_star as a_star
#import lf_node as lf_node
#import ld_node as ld_node

directions = []

def generate_directions(start_node, goal_node):
    global path
    path = a_star.astar(start_node, goal_node, a_star.graph)[0]
    return a_star.get_directions(path)


class MainNode(DTROS):
    def __init__(self, node_name):
        super(MainNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.pub = rospy.Publisher("instruction", String, queue_size=1)

    def send_instruction(self):
        rate = rospy.Rate(2) #2 message every second
        while len(directions) > 0:
            if len(directions) > 0:
                instruction = directions.pop(0)
                self.pub.publish(str(instruction))
            rate.sleep()

        

if __name__ == '__main__':
    start_node = (10, 6)
    goal_node = (10, 2)
    directions = generate_directions(start_node, goal_node)
    directions = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    # create the main node 
    node = MainNode(node_name='main_node')
    node.send_instruction()
    # run node
    rospy.spin()
