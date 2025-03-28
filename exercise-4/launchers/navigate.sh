#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launching app
rosrun apriltag safe_navigation.py
# TBA

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
