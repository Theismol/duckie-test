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
# r: 0 f: 1 l: 2 b: 3
directions = [0,0,1,1,2,2,1,1,2,2,1,1,2,2,1,1,2,2,1,1]


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
        rate = rospy.Rate(1)
        i = 0
        while not rospy.is_shutdown():
            if i == len(directions):
                i = 0
            if directions[i] == 1:
                message = WheelsCmdStamped(vel_left=FORWARD[0], vel_right=FORWARD[1])
                rospy.loginfo("f")
            elif directions[i] == 2:
                message = WheelsCmdStamped(vel_left=TURN_LEFT[0], vel_right=TURN_LEFT[1])
                rospy.loginfo("l")
            elif directions[i] == 3:
                message = WheelsCmdStamped(vel_left=BACKWARD[0], vel_right=BACKWARD[1])
                rospy.loginfo("b")
            else:
                message = WheelsCmdStamped(vel_left=TURN_RIGHT[0], vel_right=TURN_RIGHT[1])
                rospy.loginfo("r")
            i += 1
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
