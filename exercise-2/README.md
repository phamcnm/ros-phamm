# Exercise 2

The goal of this exercise is to learn to use ROS to control the movements of a Duckiebot. All the code to perform requested functions on the DuckieBot for this exercise is can be found in [packages/](https://github.com/phamcnm/ros-phamm/blob/main/exercise-2/packages/). Details of each package is below

### Commands
replace `MY_ROBOT` with `csc22936` to run our Duckiebot.
#### 1. build docker image directly on robot
`dts devel build -f -H MY_ROBOT.local`
#### 2. run the program through launchers file, based on one of the package below
`dts devel run -R ROBOT_NAME -L [LAUNCHER]`

### Packages

#### [pubsub](https://github.com/phamcnm/ros-phamm/blob/main/exercise-2/packages/pubsub/src/)
Let a publisher and subscriber communicate with each other 
`dts devel run -H ROBOT_NAME -L my-subscriber`
`dts devel run -H ROBOT_NAME -L my-publisher -n publisher`

#### [camera](https://github.com/phamcnm/ros-phamm/blob/main/exercise-2/packages/camera/src/camera_reader_node.py)
Customize and Annotate camera image
`dts devel run -R ROBOT_NAME -L camera-reader`

#### [drive_back_and_forth](https://github.com/phamcnm/ros-phamm/blob/main/exercise-2/packages/drive_back_and_forth/src/drive.py)
Let Duckiebot to drive forwards and backwards 1.25 meters
`dts devel run -R ROBOT_NAME -L drive`

#### [rotate](https://github.com/phamcnm/ros-phamm/blob/main/exercise-2/packages/rotate/src/rotate.py)
Let Duckiebot to rotate 90 clockwise then 90 counter-clockwise
`dts devel run -R ROBOT_NAME -L rotate`

#### [drive-d](https://github.com/phamcnm/ros-phamm/blob/main/exercise-2/packages/drive_d/src/drive_d.py)
Let Duckiebot travel in a D shape using LEDs to signal state
`dts devel run -R ROBOT_NAME -L drive-d`

#### 2. run the program
`docker -H MY_ROBOT.local run -it --rm --net=host duckietown/my-program:latest-arm32v7`

### Group members
Minh Pham, Alex Liu