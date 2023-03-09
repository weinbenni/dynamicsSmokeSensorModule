#!/usr/bin/python


# -------------------------------------------------------------------------------------------------------------
# Source:   Copyright Hochschule Muenchen, FK 04, Prof. Juergen Plate und die Autoren
# URL:      https://netzmafia.ee.hm.edu/skripten/hardware/RasPi/Projekt-Ultraschall/index.html - 06.03.2023
# adapted:  Weinhaeupl Benjamin -- FHOoe - Campus Wels - 06.03.2023
#--------------------------------------------------------------------------------------------------------------
import RPi.GPIO as GPIO
import time
import datetime
import rospy
from sensor_msgs.msg import Range

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
 
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
GPIO_TRIGGER6 = 11
GPIO_ECHO6 = 9

 
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

#Echo Interrupt ruecksetzen
GPIO.remove_event_detect(GPIO_ECHO1)
GPIO.remove_event_detect(GPIO_ECHO2)
GPIO.remove_event_detect(GPIO_ECHO3)
GPIO.remove_event_detect(GPIO_ECHO4)
GPIO.remove_event_detect(GPIO_ECHO5)
GPIO.remove_event_detect(GPIO_ECHO6)

time.sleep(1)                   # Setup-Zeit fuer Sensor
# Dauer Trigger-Impuls
PULSE = 0.00001

# Anzahl Messwerte fuer Mittelwertbildung
BURST = 2

# Schallgeschwindigkeit/2
SPEED_2 = 170.15

def publishSensor(sensor,measRange):
    pub = rospy.Publisher(sensor, Range, queue_size = 10)
    rospy.init_node('ultrasonic_publisher', anonymous = True)
    msg = Range()
    msg.header.frame_id = sensor
    msg.field_of_view = 0.33 #19 degrees
    msg.radiation_type = msg.ULTRASOUND
    msg.max_range = 4
    msg.min_range = 0.02
    msg.range = measRange
    
    #rospy.loginfo(msg)
    pub.publish(msg)


stopp = 0                       # Variableninit
start = 0
distance = 0

def pulse(TRIG):                    # Funktion zum Starten der Messung
  global start
  global stopp
  global distance

  GPIO.output(TRIG, True)       # Triggerimpuls  erzeugen
  time.sleep(PULSE)
  GPIO.output(TRIG, False)
  stopp = 0                     # Werte auf 0 setzen
  start = 0
  distance = 0                  # und Event starten


def measure(x):                 # Callback-Funktion fuer ECHO
  global start
  global stopp
  global distance
  if  GPIO.input(GPIO_ECHO1)==1 or GPIO.input(GPIO_ECHO2)==1 or GPIO.input(GPIO_ECHO3)==1 or GPIO.input(GPIO_ECHO4)==1 or GPIO.input(GPIO_ECHO5)==1 or GPIO.input(GPIO_ECHO6)==1 :
    start = time.time()
   # print("Started")
  #elif not GPIO.input(GPIO_ECHO1) and not GPIO.input(GPIO_ECHO2) and not GPIO.input(GPIO_ECHO3) and not GPIO.input(GPIO_ECHO4) and not GPIO.input(GPIO_ECHO5) and not GPIO.input(GPIO_ECHO6) :     # fallende Flanke, Endezeit speichern
  else: 
    stopp = time.time()
   # print("Stopped")
    delta = stopp - start       # Zeitdifferenz und Entfernung berechnen
    distance = delta * SPEED_2


def measure_range(TRIG):            # Bildet Mittelwert von BURST Messungen
  values = []
  sum = 0
  for i in range(0, BURST):
    pulse(TRIG)                     # Messung starten
    time.sleep(0.025)           # Warten, bis Messung zuende
    values.append(distance)     # Wert im Array speichern und aufsummieren
   # print("Messwert: %1.1f" % distance) # Kontrollausgabe
    sum = sum + values[i]
  return sum/BURST;             # Mittelwert zurueckgeben

# do it
try:
  GPIO.add_event_detect(GPIO_ECHO1, GPIO.BOTH, callback=measure)
  GPIO.add_event_detect(GPIO_ECHO2, GPIO.BOTH, callback=measure)
  GPIO.add_event_detect(GPIO_ECHO3, GPIO.BOTH, callback=measure)
  GPIO.add_event_detect(GPIO_ECHO4, GPIO.BOTH, callback=measure)
  GPIO.add_event_detect(GPIO_ECHO5, GPIO.BOTH, callback=measure)
  GPIO.add_event_detect(GPIO_ECHO6, GPIO.BOTH, callback=measure)

  while True:
    Dist1 = measure_range(GPIO_TRIGGER1)
    publishSensor("sonar_front_link", Dist1)
    Dist2 = measure_range(GPIO_TRIGGER2)
    publishSensor("sonar_front_left_link", Dist2)
    Dist3 = measure_range(GPIO_TRIGGER3)
    publishSensor("sonar_front_right_link", Dist3)
    Dist4 = measure_range(GPIO_TRIGGER4)
    publishSensor("sonar_back_link", Dist4)
    Dist5 = measure_range(GPIO_TRIGGER5)
    publishSensor("sonar_back_left_link", Dist5)
    Dist6 = measure_range(GPIO_TRIGGER6)
    publishSensor("sonar_back_right_link", Dist6)

# reset GPIO settings if user pressed Ctrl+C
except KeyboardInterrupt:
  print("Bye!")
  GPIO.cleanup()
