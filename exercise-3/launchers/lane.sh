#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launching app
rosrun computer_vision lane_detection_template.py
# TBA

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
