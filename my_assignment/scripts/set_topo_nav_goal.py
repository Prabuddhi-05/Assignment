#! /usr/bin/env python

import rospy # ROS library for Python
import actionlib # Library to create action clients and servers in ROS
import grape_counter as grp # Imports grape_counter.py
from sensor_msgs.msg import Image

from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

if __name__ == '__main__':
    rospy.init_node('topological_navigation_client') # Initializes a new ROS node 
    # Creates an action client to communicate with an action server
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction) 
    client.wait_for_server()

    waypoints = ["WayPoint0","WayPoint1","WayPoint2","WayPoint3","WayPoint4","WayPoint5"] #List of goal targets 

    counter=grp.grape_counter()
    final_count=0
    
    for waypoint in waypoints:
        goal = GotoNodeGoal() # Creates a new goal
        goal.target = waypoint
        rospy.loginfo("Going to %s", goal.target)
        client.send_goal(goal) # Goal is sent to the robot
        status = client.wait_for_result() # Waits until the action is complete (Waits for the robot to reach the goal target)
        result = client.get_result()
        rospy.loginfo("Status is %s", status)
        rospy.loginfo("Result is %s", result)
        if result.success: # If the robot reaches the goal target
            data = rospy.wait_for_message("/thorvald_001/kinect2_front_camera/hd/image_color_rect", Image) # Gets the image data
            counter.get_grape_count(data)
            rospy.loginfo("Grape bunch count at %s: %d",waypoint, counter.grapes_count) # Prints the grape bunch count at each waypoint
            final_count += counter.grapes_count
    rospy.loginfo("Total grape bunch count : %d",final_count) # Prints the total grape bunch count in the vineyard    