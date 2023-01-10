# CMP9767 – Robot Programming (Assessment 1)

## Solution

## How to arrange the ROS setup before running the system
1. Create a catkin workspace called `catkin_ws` and create a src directory by issuing the command `mkdir -p ~/catkin_ws/src` in the terminal
2. Update the system through `update system sudo apt-get update && sudo apt-get upgrade`
3. In the source directory, `catkin_ws/src`, install all the dependencies `rosdep install --from-paths . -i -y`
4. Installl scikit-learn thorugh `sudo apt-get install python3-sklearn python3-sklearn-lib`
5. Fork the workshop repository to `catkin_ws/src` through the command `git clone https://github.com/LCAS/CMP9767M.git` 
6. Create a folder called `Assignment` in `catkin_ws/src` and create a catkin package called `my_assignment` in `catkin_ws/src/Assignment` through `catkin_create_pkg my_assignment roscpp std_msgs cv_bridge`   
7. Fork the contents of the `Assignmnent` repository in GitHub to `catkin_ws/src/Assignment/my_assignment` through `git clone https://github.com/Prabuddhi-05/Assignment.git`
8. Move to the `catkin_ws` and issue the command `catkin_make` 
9. Source the workspace through the command `source devel/setup.bash`

For more information, refer : `https://github.com/LCAS/CMP9767M/wiki/Typical-workflows`

## How to start the system
Launch the system through `roslaunch my_assignment grape_counter.launch`
