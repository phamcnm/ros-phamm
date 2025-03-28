# Exercise 4

The goal of this exercise is to learn to use Ros, vision, odometry, all together to perform close control with the DuckieBot in simple traffic environments. All the code to perform requested functions on the DuckieBot for this exercise can be found in [packages/](https://github.com/phamcnm/ros-phamm/blob/main/exercise-4/packages/). Details of each package is below.

### Commands
Replace `MY_ROBOT` with `csc22936` to run our Duckiebot.
#### 1. build docker image directly on robot
`dts devel build -f -H MY_ROBOT.local`
#### 2. run the program through launchers file, based on one of the package below
`dts devel run -R ROBOT_NAME -L [LAUNCHER]`

### Packages

The structure of the code is: Each script corresponds to each task in this exercise. They share the same package but each script is run using a separate launcher.

#### [apriltag_detection](https://github.com/phamcnm/ros-phamm/blob/main/exercise-4/packages/apriltag/src/apriltag_detection.py)
April Tag Detection and Respond to Traffic Signs

`dts devel run -H ROBOT_NAME -L apriltag` 

#### [crosswalk](https://github.com/phamcnm/ros-phamm/blob/main/exercise-4/packages/apriltag/src/crosswalk.py)
Stop at crossroads and detect pedestrians

`dts devel run -R ROBOT_NAME -L crosswalk`

#### [safe_navigation](https://github.com/phamcnm/ros-phamm/blob/main/exercise-4/packages/apriltag/src/safe_navigation.py)
Let Duckiebot detect any idle cars and pass them

`dts devel run -R ROBOT_NAME -L navigate`

### Group members
Minh Pham, Alex Liu