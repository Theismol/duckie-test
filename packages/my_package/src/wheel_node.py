#!/usr/bin/env python3
from random import uniform
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String


TURN_RIGHT = (0.17,0)
FORWARD = (0.2,0.23)
BACKWARD = (-0.2,-0.26)
TURN_LEFT = (0,0.17)
STOP = (0,0)

#directions = a_star.get_directions(a_star.astar(start_node, goal_node, a_star.graph)[0])

class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._forward = FORWARD
        self._turn_left = TURN_LEFT
        self._turn_right = TURN_RIGHT
        self._backward = BACKWARD
        self._stop = STOP
        #DET HER ER COPILOT MÅSKE KAN VI TESTE HVAD DER ER HER. KAN VÆRE DET ER DEN KALIBREREDE VÆRDI
        #self._forward = rospy.get_param(f'/{vehicle_name}/kinematics_node/forward')
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self._subsriber = rospy.Subscriber("lane_detection_correction", String, self.callback)
        #create a new subscriber with inger as the message type
        self._subsriber2 = rospy.Subscriber("instruction", String, self.run)

    def run(self,instruction):
        instruction = int(instruction.data)
        rate = rospy.Rate(1) #1 message every second
        if instruction == 1:
            message = WheelsCmdStamped(vel_left=self._forward[0] , vel_right=self._forward[1])
            rospy.loginfo("f")
        elif instruction == 2:
            message = WheelsCmdStamped(vel_left=self._turn_left[0], vel_right=self._turn_left[1])
            rospy.loginfo("l")
        elif instruction == 3:
            message = WheelsCmdStamped(vel_left=self._backward[0], vel_right=self._backward[1])
            rospy.loginfo("b")
        else:
            message = WheelsCmdStamped(vel_left=self._turn_right[0], vel_right=self._turn_right[1])
            rospy.loginfo("r")
        self._publisher.publish(message)
        rate.sleep()
        
        # rate = rospy.Rate(1) #1 message every second
        # i = 0
        # while not rospy.is_shutdown():
        #     if i == len(directions):
        #         rospy.loginfo("arrayed at tarked")
        #         i = 0
        #     if directions[i] == 1:
        #         message = WheelsCmdStamped(vel_left=self._forward[0], vel_right=self._forward[1])
        #         rospy.loginfo("f")
        #     elif directions[i] == 2:
        #         message = WheelsCmdStamped(vel_left=self._turn_left[0], vel_right=self._turn_left[1])
        #         rospy.loginfo("l")
        #     elif directions[i] == 3:
        #         message = WheelsCmdStamped(vel_left=self._backward[0], vel_right=self._backward[1])
        #         rospy.loginfo("b")
        #     else:
        #         message = WheelsCmdStamped(vel_left=self._turn_right[0], vel_right=self._turn_right[1])
        #         rospy.loginfo("r")
        #     i += 1
        #     self._publisher.publish(message)
        #     rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

    def callback(self, data):
        rospy.loginfo("WE ARE IN CALLBACK")
        print(data.data)
        #if data.data == "right":
        #    self._forward = (0.11,0.010)
        #    print("right WE ARE IN CALLBACK")
        #elif data.data == "left":
        #    self._forward = (0.10,0.095)
        #    print("left WE ARE IN CALLBACK")
        #elif data.data == "no_change":
        #    self._forward = FORWARD
        #    print("no_change WE ARE IN CALLBACK")


if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    rospy.spin()
