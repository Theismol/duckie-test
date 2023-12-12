#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package ld_node.py &
rosrun my_package wheel_node.py 

# wait for app to end
dt-launchfile-join
