#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun reverse_park reverse.py

# wait for app to end
dt-launchfile-join
