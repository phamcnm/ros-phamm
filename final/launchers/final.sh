#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launching app
rosrun final final.py --zone 2
# TBA

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
