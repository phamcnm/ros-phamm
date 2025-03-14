# Exercise 3

The goal of this exercise is to incorporate DuckieBot vision and wheel movements. All the code to perform requested functions on the DuckieBot for this exercise can be found in [packages/](https://github.com/phamcnm/ros-phamm/blob/main/exercise-3/packages/). Details of each package is below.

### Commands
Replace `MY_ROBOT` with `csc22936` to run our Duckiebot.
#### 1. build docker image directly on robot
`dts devel build -f -H MY_ROBOT.local`
#### 2. run the program through launchers file, based on one of the package below
`dts devel run -H ROBOT_NAME -L [LAUNCHER]`

### Packages

#### [undistort](https://github.com/phamcnm/ros-phamm/blob/main/exercise-3/packages/cv_undistort/src/)
Undistort the image seen from the camera

`dts devel run -H ROBOT_NAME -L cv_undistort` 

#### [lane_based_behavior](https://github.com/phamcnm/ros-phamm/blob/main/exercise-3/packages/computer_vision/src/lane_based_behavior_controller_template.py)
Perform requested vision-wheel behaviors

`dts devel run -H ROBOT_NAME -L detect`

#### [lane_following](https://github.com/phamcnm/ros-phamm/blob/main/exercise-3/packages/computer_vision/src/lane_detection_template.py)
Perform lane following. This program fulfills the requirements of both part 2 and 3.

`dts devel run -H ROBOT_NAME -L lane`

### Group members
Minh Pham, Alex Liu