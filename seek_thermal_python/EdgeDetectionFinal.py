#!/usr/bin/env python
#https://www.youtube.com/watch?v=esmpHCJz8xI
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np
#import sys
#import roslib


# Instantiate CvBridge
bridge = CvBridge()
# Define a subsciber
rospy.init_node('image_listener')
# Define your image topic
image_topic1 = "/seek/left/image_grey"

def nothing(x):
    pass

# open a new window for the Trackbar
cv2.namedWindow("Trackbar")
# create two new Trackbars
cv2.createTrackbar("Threshold1_Upper", "Trackbar", 0, 255, nothing)
cv2.createTrackbar("Threshold1_Lower", "Trackbar", 0, 255, nothing)
# set tresholds to initial values
upperThreshold1_Start = 27
lowerThreshold1_Start = 0
cv2.setTrackbarPos("Threshold1_Upper", "Trackbar", upperThreshold1_Start)
cv2.setTrackbarPos("Threshold1_Lower", "Trackbar", lowerThreshold1_Start)

def image_callback(msg):
    #print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2 ("bgr8")
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")


        show_images(cv2_img)

    except CvBridgeError as e:
        print("CvBridge Error: {0}".format(e))

def show_images(img):
     # Flip the image 90deg
     cv_image = cv2.transpose(img)
     cv_image = cv2.flip(cv_image,1)
     # Get image hight and width / to resize with same ratio
     dimension = cv_image.shape
     hight = dimension[0]
     width = dimension[1]
     resizeFactor = 2

     # Get Trackbar positions to regulate upper and lower threshold values
     upperThreshold1 = cv2.getTrackbarPos("Threshold1_Upper", "Trackbar")
     lowerThroshold1 = cv2.getTrackbarPos("Threshold1_Lower", "Trackbar")
     
     # resize the cv_image, the resulting image is now base for all filtering processes
     frame = cv2.resize(cv_image, (resizeFactor*width, resizeFactor*hight))  
     cv2.imshow("Original frame", frame)

     gaussian = cv2.GaussianBlur(frame, (5, 5), cv2.BORDER_DEFAULT)     
     edge1 = cv2.Canny(frame, lowerThroshold1, upperThreshold1, edges=3)     
     edge2 = cv2.Canny(gaussian, lowerThroshold1, upperThreshold1, edges=3)  
     dilated1 = cv2.dilate(edge1, (1, 1), iterations=2)
     dilated2 = cv2.dilate(edge2, (1, 1), iterations=2)

     _, contours1, _ = cv2.findContours(dilated1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
     cv2.drawContours(frame, contours1, -1, (0, 255, 0), 2)

     _, contours2, _ = cv2.findContours(dilated2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
     #cv2.drawContours(gaussian, contours2, -1, (0, 255, 0), 2)

     # make the same dimensions
     edged1 = np.stack((edge1,) * 3, axis = -1)
     edged2 = np.stack((edge2,) * 3, axis = -1)

     images = [frame, gaussian, edged1, edged2]
     win_names = ['Grey', 'Gaussian', 'Canny', 'Gaussion+Canny']

     font = cv2.FONT_HERSHEY_SIMPLEX

     # Horizontal window stacking
     img_stack = np.hstack(images)
     for index, name in enumerate(win_names):
         image = cv2.putText(img_stack, "{Index}. {Name}".format(Index = index + 1, Name = name), (5 + frame.shape[1] * index, 30), font, 1, (205, 0, 255), 2, cv2.LINE_AA)

     cv2.imshow("Image Processing", img_stack)
    

    # very important, otherwise programm does not work
     cv2.waitKey(3)
     

rospy.Subscriber(image_topic1, Image, image_callback)
rospy.spin()

    
  
        
       