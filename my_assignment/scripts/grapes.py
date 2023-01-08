#!/usr/bin/env python

# Python Libraries
import sys, time
import numpy as np
from sklearn.cluster import DBSCAN
import poles as pl

# OpenCV
import cv2

# Ros Libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

class grape_counter:
    camera_model = None
    image_depth_ros = None

    visualisation = True
    # Aspect ratio between color and depth cameras
    # Calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the kinectv2 urdf file
    # (84.1/1920) / (70.0/512)
    color2depth_aspect = (84.1/1920) / (70.0/512)

    def __init__(self):    

        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_front_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        self.object_location_pub = rospy.Publisher('/thorvald_001/object_location', PoseStamped, queue_size=10)

        rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect",
            Image, self.image_color_callback)

        rospy.Subscriber("/thorvald_001/kinect2_front_sensor/sd/image_depth_rect",
            Image, self.image_depth_callback)
            
        self.image_pub = rospy.Publisher("/grape_counter/image", Image, queue_size=1) 
        
        self.count_pub = rospy.Publisher("/grape_counter/count", Int32, queue_size=10) 
        self.rate = rospy.Rate(10)

        self.tf_listener = tf.TransformListener()

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() # Only subscribes once

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        # Waits for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # Converts images to OpenCV
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)
    
        # Detects of grapes in the color image
        image_mask = cv2.inRange(image_color, (100, 100, 100), (190, 190, 190)) 
        image_mask = cv2.bitwise_not(image_mask)
        masked_image = cv2.bitwise_and(image_color, image_color, mask = image_mask)

        b,g,r = cv2.split(masked_image)
        new_image = 2*b - g - r
        image_mask = cv2.inRange(new_image, 10, 90)
        masked_image = cv2.bitwise_and(new_image, new_image, mask = image_mask)

        threshold = 50 
        binary_image = cv2.threshold(masked_image, threshold, 255, cv2.THRESH_BINARY)[1]

        kernel = np.ones((5, 5), np.uint8)
        dilated_image = cv2.dilate(binary_image, kernel, iterations=5)  

        contours, hierarchy = cv2.findContours(dilated_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        centroids = []

        # Calculates the moments of each contour
        for c in contours:
            M = cv2.moments(c)
        
        # Calculates the x,y coordinates of the centers of each contour
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]
            centroids.append([cX, cY])
   
        epsilon_threshold = 75.0
        db = DBSCAN(eps=epsilon_threshold, min_samples=1).fit(centroids)
        labels = db.labels_
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        print("Number of grapes: %d" % n_clusters_)
    
        for i in range(n_clusters_):
            j=np.where(labels == i)
            [cX, cY] = db.components_[j[0][0]]
            cv2.circle(image_color,(int(cX),int(cY)), 5, (0, 0, 255), -1)
    
        if M["m00"] == 0:
            print('No object detected.')
            
        # Calculates the y,x centroid
        #image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
        image_coords = (cY, cX)
        # "map" from color to depth image
        depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect, 
        image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect)
        # get the depth reading at the centroid location
        depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!

        print('image coords: ', image_coords)
        print('depth coords: ', depth_coords)
        print('depth value: ', depth_value)

        # Calculates the object's 3D location in camera coordinates
        camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) # Projects the image coordinates (x,y) into 3D ray in camera coordinates 
        camera_coords = [x/camera_coords[2] for x in camera_coords] # Adjusts the resulting vector so that z = 1
        camera_coords = [x*depth_value for x in camera_coords] # Multiplies the vector by depth

        #print('camera coords: ', camera_coords)

        # Defines a point in camera coordinates
        object_location = PoseStamped()
        object_location.header.frame_id = "thorvald_001/kinect2_front_rgb_optical_frame"
        object_location.pose.orientation.w = 1.0
        object_location.pose.position.x = camera_coords[0]
        object_location.pose.position.y = camera_coords[1]
        object_location.pose.position.z = camera_coords[2]

        # Publishes so we can see that in RVIZ
        self.object_location_pub.publish(object_location)
        
        #Prints out the coordinates in the map frame
        p_camera = self.tf_listener.transformPose('map', object_location)

        print('map coords: ', p_camera.pose.position)
        print('')

        if self.visualisation:
            # Drawing circles
            #cv2.circle(image_color, (int(image_coords[1]), int(image_coords[0])), 10, 255, -1)
            cv2.circle(image_depth, (int(depth_coords[1]), int(depth_coords[0])), 5, 255, -1)

            # Resizes and adjusts for visualisation
            image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
            image_depth *= 1.0/10.0 # Scale for visualisation (max range 10.0 m)
            #dilated_image = cv2.resize(dilated_image, (0,0), fx=0.5, fy=0.5)

            cv2.imshow("image depth", image_depth)
            cv2.imshow("image color", image_color)
            #cv2.imshow("dilated image",dilated_image)
            cv2.waitKey(1)
            
            ros_image = self.bridge.cv2_to_imgmsg(image_color, "bgr8")
            self.image_pub.pubish(ros_image)
            self.count_pub.publish(n_clusters_)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('grape_counter', anonymous=True)
    ic = grape_counter()
    pl.pole_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
