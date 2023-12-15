#!/usr/bin/env python3
from random import uniform
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String
import lf_node as lf
import main



STOP = (0,0)

#directions = a_star.get_directions(a_star.astar(start_node, goal_node, a_star.graph)[0])
class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self.updated_way = (0.30, 0.33)
        self.current_way = (0.30, 0.33)
        self.intersection = False
        self._stop = STOP
        #DET HER ER COPILOT MÅSKE KAN VI TESTE HVAD DER ER HER. KAN VÆRE DET ER DEN KALIBREREDE VÆRDI
        #self._forward = rospy.get_param(f'/{vehicle_name}/kinematics_node/forward')
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self._subscriber = rospy.Subscriber("lane_detection_correction", String, self.callback)

    def test(self):
        while not rospy.is_shutdown():
            if self.intersection == True:
                self.intersectionGet()
                continue
            rate = rospy.Rate(2) #1 message every second__
            self.updated_way = self.current_way
            print("\n")
            print("------Updated instruction-------", self.updated_way)
            print("\n")
            message = WheelsCmdStamped(vel_left=self.updated_way[0], vel_right=self.current_way[1])
            self._publisher.publish(message)
            rate.sleep()
            rate = rospy.Rate(0.5)
            self.updated_way = self.current_way
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            rate.sleep()



    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

    def callback(self, data):
        get_data = data.data
        #splist the date so there is only the numbers
        split_data = get_data.split(",")
        pontsOfLines = []
        #add int to the list
        for i in split_data:
            pontsOfLines.append(int(i))

        listOfWays = []
        #range of 5
        index = 0
        if self.intersection == False:
            for i in range(10):
                wayAndColor = lf.main(pontsOfLines, self.updated_way)
                listOfWays.append(wayAndColor)
                index += 1
                
            for i in range(10):
                if listOfWays[i][1] == "red" and self.intersection == False:
                    self.intersection = True
                    break
                    #start intersection: 
            self.current_way = listOfWays[0][0]

    def intersectionGet(self):
        print("------Intersection-------")
        rate = rospy.Rate(1) #1 message every second
        if main.get_directions() == "r":
            main.remove_first()
            self.updated_way = (0.35, 0.0)
            rate.sleep()
            message = WheelsCmdStamped(vel_left=0.4, vel_right=0.4)
            self._publisher.publish(message)
            rate.sleep()
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            rate.sleep()
            message = WheelsCmdStamped(vel_left=self.updated_way[0], vel_right=self.updated_way[1])
            self._publisher.publish(message)
            rate.sleep()
        elif main.get_directions() == "l":
            main.remove_first()
            self.updated_way = (0.36, 0.37)
            message = WheelsCmdStamped(vel_left=self.updated_way[0], vel_right=self.updated_way[1])
            self._publisher.publish(message)
            rate.sleep()
            rate.sleep()
            message = WheelsCmdStamped(vel_left=0, vel_right=0.3)
            self.updated_way = (0.35, 0.3)
            self._publisher.publish(message)
            rate.sleep()
            message = WheelsCmdStamped(vel_left=self.updated_way[0], vel_right=self.updated_way[1])
            self._publisher.publish(message)
            rate.sleep()
        if main.get_directions() == "s":
            self.intersection = True
            #forrword 0.4 0.4
            message = WheelsCmdStamped(vel_left=0.4, vel_right=0.4)
            self._publisher.publish(message)
            rate.sleep()
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            rate.sleep()
            #shutdowm ros
            print("----the Robot arrived at the destination----")
            rospy.signal_shutdown("Shutting down")
        else:
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            rate.sleep()
            rate.sleep()
            rate.sleep()
            rate.sleep()
            self.intersection = False



if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    node.test()
    rospy.spin()
