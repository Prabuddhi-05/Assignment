# CMP9767 – Robot Programming (Assessment 1)
## Prabuddhi Wariyapperuma (Student ID: 26619055)

## Solution

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
