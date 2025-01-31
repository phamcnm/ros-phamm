# Exercise 1

The code for this exercise is can be found in [packages/lab_exercises/exercise_1_script.py](https://github.com/phamcnm/ros-phamm/blob/main/exercise-1/packages/lab_exercises/exercise_1_script.py). The goal of this exercise is to deploy a software directly on a Duckiebot and run it.
 
### Commands
replace `MY_ROBOT` with `csc22936` to run our Duckiebot.
#### 1. build docker image directly on robot
`dts devel build -f --arch arm32v7 -H MY_ROBOT.local`

#### 2. run the program
`docker -H MY_ROBOT.local run -it --rm --net=host duckietown/my-program:latest-arm32v7`

### Collaborators
Alex Liu