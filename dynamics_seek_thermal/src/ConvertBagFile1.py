#!
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
#import sys
#import roslib


# Instantiate CvBridge
bridge = CvBridge()
# Define a subsciber
rospy.init_node('image_listener')
# Define your image topic
image_topic1 = "/seek/left/image_grey"

def image_callback(msg):
    print("Received an image!")
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
     
     frame = cv2.resize(cv_image, (resizeFactor*width, resizeFactor*hight))
     cv2.imshow("Frame", frame)
     cv2.moveWindow("Frame", 0, 0)

     gaussian = cv2.GaussianBlur(frame, (5, 5), cv2.BORDER_DEFAULT)
     cv2.imshow("Gauss", gaussian)
     cv2.moveWindow("Gauss", resizeFactor*width, 0)

     edge1 = cv2.Canny(frame, 20, 30)
     cv2.imshow("Edge Normal", edge1)
     cv2.moveWindow("Edge Normal", 2*resizeFactor*width, 0)

     edge2 = cv2.Canny(gaussian, 40, 50)
     cv2.imshow("Edge Gauss", edge2)
     cv2.moveWindow("Edge Gauss", 3*resizeFactor*width, 0)

    # very important, otherwise programm does not work
     cv2.waitKey(3)

#def main():
    
try:
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic1, Image, image_callback)
    rospy.spin()
        
except KeyboardInterrupt:
    print("Ende")
    
        #while not rospy.is_shutdown():
        #  rospy.spin()
       

# Initialize an OpenCV Window named "Image Window"
#cv2.namedWindow("Image Window", 1)
#main()