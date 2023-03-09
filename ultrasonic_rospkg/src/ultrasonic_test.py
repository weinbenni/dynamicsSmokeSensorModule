#!/usr/bin/env python

#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import rospy
from sensor_msgs.msg import Range
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins1
GPIO_TRIGGER1 = 14
GPIO_ECHO1 = 15
#set GPIO Pins2
GPIO_TRIGGER2 = 23
GPIO_ECHO2 = 24
#set GPIO Pins3
GPIO_TRIGGER3 = 25
GPIO_ECHO3 = 8
#set GPIO Pins4
GPIO_TRIGGER4 = 7
GPIO_ECHO4 = 12
#set GPIO Pins5
GPIO_TRIGGER5 = 26
GPIO_ECHO5 = 19
#set GPIO Pins6
GPIO_TRIGGER6 = 9
GPIO_ECHO6 = 11

 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER1, GPIO.OUT)
GPIO.setup(GPIO_ECHO1, GPIO.IN)
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER2, GPIO.OUT)
GPIO.setup(GPIO_ECHO2, GPIO.IN)
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER3, GPIO.OUT)
GPIO.setup(GPIO_ECHO3, GPIO.IN)
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER4, GPIO.OUT)
GPIO.setup(GPIO_ECHO4, GPIO.IN)
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER5, GPIO.OUT)
GPIO.setup(GPIO_ECHO5, GPIO.IN)
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER6, GPIO.OUT)
GPIO.setup(GPIO_ECHO6, GPIO.IN)
 
def distance(GPIO_TRIGGER, GPIO_ECHO):
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 343) / 2
 
    return distance
 
def publishSensor(sensor,measRange):
    pub = rospy.Publisher(sensor, Range, queue_size = 10)
    rospy.init_node('ultrasonic_publisher', anonymous = True)
    rate = rospy.Rate(3)
    msg = Range()
    msg.header.frame_id = sensor
    msg.field_of_view = 50/146.0*3.14159
    msg.radiation_type = msg.ULTRASOUND
    msg.max_range = 4
    msg.min_range = 0.02
    msg.range = measRange
    
    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()

 
if __name__ == '__main__':
    try:
        while True:
            try:
                rospy.get_master().getPid()

                dist1 = distance(GPIO_TRIGGER1,GPIO_ECHO1)
                publishSensor("sonar_front_link", dist1 )
                dist2 = distance(GPIO_TRIGGER2,GPIO_ECHO2)
                publishSensor("sonar_front_right_link", dist2 )
                dist3 = distance(GPIO_TRIGGER3,GPIO_ECHO3)
                publishSensor("sonar_front_left_link", dist3 ) 
                dist4 = distance(GPIO_TRIGGER4,GPIO_ECHO4)
                publishSensor("sonar_back_link", dist4 ) 
                dist5 = distance(GPIO_TRIGGER5,GPIO_ECHO5)
                publishSensor("sonar_back_left_link", dist5 ) 
                dist6 = distance(GPIO_TRIGGER6,GPIO_ECHO6)
                publishSensor("sonar_back_right_link", dist6 ) 
            
            except:
                failed = True
                print("No connection to roscore")

           
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
