#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun drive_back_and_forth drive.py

# wait for app to end
dt-launchfile-join