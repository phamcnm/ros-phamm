#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun drive_d drive_d.py

# wait for app to end
dt-launchfile-join