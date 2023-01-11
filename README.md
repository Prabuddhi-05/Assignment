# CMP9767 – Robot Programming (Assessment 1)
## Prabuddhi Wariyapperuma (Student ID: 26619055)

## Solution
In this assignment, a software system for the Thorvald robot (an agricultural robot) that is autonomously navigating in a simulated vineyard is implemented using Robot Operating System (ROS) platform and OpenCV. Further, the robot is also capable of detecting and counting the grape bunches in the vineyard.
A topological map is used to navigate the robot to different waypoints. Important operations in OpenCV such as masking, image thresholding, morphological transformations such as erosion and dilation and a clustering algorithm called density-based spatial clustering of applications with noise (DBSCAN) were used for the detection of the grape bunches.   

## How to arrange the ROS setup before running the system
1. Create a catkin workspace called `catkin_ws` and create an `src` directory by issuing the command `mkdir -p ~/catkin_ws/src`
2. In the source directory `catkin_ws/src`, update the system through `sudo apt-get update && sudo apt-get upgrade`
3. In `catkin_ws/src`, install all the dependencies `rosdep install --from-paths . -i -y`
4. Installl scikit-learn thorugh `sudo apt-get install python3-sklearn python3-sklearn-lib`
5. Fork the workshop repository to `catkin_ws/src` through the command `git clone https://github.com/LCAS/CMP9767M.git`
6. Copy the contents of the zip file `Assignment` (after extracted) to `catkin_ws/src`
7. OR instead of Step 6, fork the contents of the `Assignment` repository in GitHub to `catkin_ws/src` through `git clone https://github.com/Prabuddhi-05/Assignment.git`
9. Move to the `catkin_ws` and issue the command `catkin_make` 
10. Source the workspace through the command `source devel/setup.bash`

References : `https://github.com/LCAS/CMP9767M/wiki/Typical-workflows`

## How to start the system
Launch the system through `roslaunch my_assignment grape_counter.launch`
