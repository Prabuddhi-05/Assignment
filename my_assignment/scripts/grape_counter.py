#!/usr/bin/env python

# Python Libraries
import sys, time
import numpy as np
from sklearn.cluster import DBSCAN # To use DBSCAN clustering algorithm to identify clusters

# OpenCV
import cv2

# ROS Libraries
import roslib, rospy

# ROS Messages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class grape_counter:
    visualisation = True

    def __init__(self):    

        self.bridge = CvBridge() # Provides an interface for converting between ROS messages and OpenCV messages

        # Creates a subscriber to listen to the images (in color format) from the Kinect 2 front camera
        rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect", Image, self.image_color_callback)

    def image_color_callback(self, data):
        # Returns an OpenCV image in the format of 8-bit, 3-channel (Blue,Green,Red)
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def get_grape_count(self,data):
        # Returns an OpenCV image in the format of 8-bit, 3-channel (Blue,Green,Red)
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Detects the poles in the color image
        
        image_mask = cv2.inRange(image_color, (0, 0, 0), (50, 50, 50)) # Removes the ground from the color image (blackens the ground)    
        masked_image = cv2.bitwise_and(image_color, image_color, mask = image_mask) # Applies a mask on the input color image
       
        image_hsv = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)  # Converts the image from RGB color space to HSV (Hue, Saturation, Value) color space
        h,s,v = cv2.split(image_hsv) # Splits the converted image to individual H,S,V channels 
        image_mask = cv2.inRange(h,17,19) 
        masked_image = cv2.bitwise_and(h,h,mask=image_mask) # Isolates the poles (in white)

        threshold = 17 
        image_binary = cv2.threshold(masked_image, threshold, 255, cv2.THRESH_BINARY)[1] # Creates a binary image by thresholding
        image_gaus = cv2.GaussianBlur(image_binary, (5,5), 1) # Removes noise from the image
        kernel = np.ones((5, 5), np.uint8) # Structuring element
        image_eroded= cv2.erode(image_gaus, kernel, iterations=3) # Discards pixels near the boundary 
        image_dilated = cv2.dilate(image_eroded, kernel, iterations=5) # Adds pixels near the boundary 
        
        threshold = 10
        image_binary = cv2.threshold(image_dilated, threshold, 255, cv2.THRESH_BINARY)[1] # Applies more thresholding

        contours, hierarchy = cv2.findContours(image_binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # Finds the contours of the image 
        centroids = [] # Empty list

        # Calculates the moments of each contour
        for c in contours:
            M = cv2.moments(c)
        
        # Calculates the x,y coordinates of the centers of each contour
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centroids.append([cX, cY]) # Appends cX and cY to the centroids list
        
        epsilon_thres = 300.0
        db = DBSCAN(eps=epsilon_thres, min_samples=1).fit(centroids) # Uses the DBSCAN algorithm to find clusters 
        labels = db.labels_ # An array of labels of clusters
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0) # Number of clusters 
        
        poles = [] # Empty list for poles
        for i in range(n_clusters_): # Iterates through each cluster
        	j = np.where(db.labels_ == i) # Finds the indices of the points that belong to the same cluster 
        	cX = db.components_[j[0][0]][0] # Selects the 1st point in the cluster 
        	poles.append([cX]) # Appends cX to the poles list
        	cv2.line(image_color,(cX, 0),(cX, 1024),(0, 255, 0), 2) # Draws lines at the coordinates of the poles
    
        # Detects the grape bunches in the color image

        image_hsv = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV) # Converts the image from RGB color space to HSV (Hue, Saturation, Value) color space
        h,s,v = cv2.split(image_hsv) # Splits the converted image to individual H,S,V channels 
        image_mask = cv2.inRange(h,100,110)
        masked_image = cv2.bitwise_and(h,h,mask=image_mask) # Isolates the grape bunches (in white)

        threshold = 50 
        image_binary = cv2.threshold(masked_image, threshold, 255, cv2.THRESH_BINARY)[1]  # Creates a binary image by thresholding
        kernel = np.ones((5, 5), np.uint8) # Structuring element 
        image_dilated = cv2.dilate(image_binary, kernel, iterations=2) # Adds pixels near the boundary 
        image_eroded = cv2.dilate(image_dilated, kernel, iterations=1) # Discards pixels near the boundary  

        contours, hierarchy = cv2.findContours(image_eroded,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # Finds the contours of the image 
        centroids = [] # Empty list

        # Calculates the moments of each contour
        for c in contours:
            M = cv2.moments(c)
        
        # Calculates the x,y coordinates of the centers of each contour
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centroids.append([cX, cY]) # Appends cX and cY to the centroids list
   
        epsilon_threshold = 30.0
        db = DBSCAN(eps=epsilon_threshold, min_samples=1).fit(centroids) # Uses the DBSCAN algorithm to find clusters 
        labels = db.labels_ # An array of labels of clusters
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0) # Number of clusters

        self.grapes_count = 0

        lower = min(poles) # The minimum cX value of poles  
        upper = max(poles) # The maximum cX value of poles 
    
        for i in range(n_clusters_): # Iterates through each cluster
            j=np.where(labels == i) # Finds the indices of the points that belong to the same cluster
            [cX, cY] = db.components_[j[0][0]] # Selects the 1st point in the cluster  
            if lower <= cX <= upper: # If cX is in the middle of lower and upper poles (To avoid double counting of grape bunches)
                self.grapes_count+=1 
                cv2.circle(image_color, (cX, cY), 5, (255, 0, 0), -1) # Draws a blue circle at cX, cY 
            else:
                cv2.circle(image_color, (cX, cY), 5, (0, 0, 255), -1) # Draws a red circle at cX, cY 

        #print("Grape bunch count",self.grapes_count)
    
        if M["m00"] == 0:
            print('No object detected.')
            
        if self.visualisation:
            image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5) # Resizes the image 
            cv2.imshow("image color", image_color) # Displays the image
            cv2.waitKey(100) # Waits for 100 milliseconds for a key event
        return 0