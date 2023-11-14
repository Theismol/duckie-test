#!/usr/bin/env python3
from random import uniform
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped


# throttle and direction for each wheel
THROTTLE_LEFT = 0.4        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.43       # 30% throttle
DIRECTION_RIGHT = 1       # backward

TURN_RIGHT = (0.1,0)
FORWARD = (0.1,0.13)
BACKWARD = (-0.1,-0.13)
TURN_LEFT = (0,0.1)
directions = ['r','f','f','l','f','f','l','f','f','l','f','f']


class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        # form the message
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(0.8)
        while not rospy.is_shutdown():
            for direction in directions:
                if direction == 'f':
                    message = WheelsCmdStamped(vel_left=FORWARD[0], vel_right=FORWARD[1])
                elif direction == 'l':
                    message = WheelsCmdStamped(vel_left=TURN_LEFT[0], vel_right=TURN_LEFT[1])
                elif direction == 'b':
                    message = WheelsCmdStamped(vel_left=BACKWARD[0], vel_right=BACKWARD[1])
                elif direction == 'r':
                    message = WheelsCmdStamped(vel_left=TURN_RIGHT[0], vel_right=TURN_RIGHT[1])
                self._publisher.publish(message)
                rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
