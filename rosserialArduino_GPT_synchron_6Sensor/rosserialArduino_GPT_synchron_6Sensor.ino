#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h> // Bibliothek zur Verwendung von HC-SR04-Sensoren

ros::NodeHandle nh;


char frameid[] = "sonar_0";
const int numSensors = 6; // Anzahl der Ultraschallsensoren
#define MAX_DISTANCE 400 // Maximale Entfernung in cm, die gemessen werden kann

// Array von NewPing-Objekten zur Verwendung der HC-SR04-Sensoren
NewPing sonar[numSensors] = {
  NewPing(2, 3, MAX_DISTANCE),
  NewPing(4, 5, MAX_DISTANCE),
  NewPing(6, 7, MAX_DISTANCE),
  NewPing(8, 9, MAX_DISTANCE),
  NewPing(10, 11, MAX_DISTANCE),
  NewPing(12, 13, MAX_DISTANCE)
};


sensor_msgs::Range distanceMsgs[numSensors]; // Nachrichten zum Veröffentlichen der Entfernungen
ros::Publisher distancePubs[numSensors] = { // Publisher zum Veröffentlichen der Entfernungen auf den rostopics
    ros::Publisher("/sonar_front_link", &distanceMsgs[0]),
    ros::Publisher("/sonar_front_left_link", &distanceMsgs[1]),
    ros::Publisher("/sonar_back_left_link", &distanceMsgs[2]),
    ros::Publisher("/sonar_front_right_link", &distanceMsgs[3]),
    ros::Publisher("/sonar_back_right_link", &distanceMsgs[4]),
    ros::Publisher("/sonar_back_link", &distanceMsgs[5])
};

volatile long startTime[numSensors]; // Startzeit des Echo-Signals
volatile long endTime[numSensors]; // Endzeit des Echo-Signals

// Interrupt Service Routinen zum Speichern der Endzeit des Echo-Signals
void echoISR0() { endTime[0] = micros(); }
void echoISR1() { endTime[1] = micros(); }

typedef void (*echoISR)();
echoISR echoISRs[numSensors] = {echoISR0, echoISR1};

void setup() {
    nh.initNode(); // Initialisiere den ROS-Node
    for (int i = 0; i < numSensors; i++) {
        //pinMode(triggerPins[i], OUTPUT); // Setze Trigger-Pins als Ausgänge
        //pinMode(echoPins[i], INPUT); // Setze Echo-Pins als Eingänge
        //attachInterrupt(digitalPinToInterrupt(echoPins[i]), echoISRs[i], FALLING); // Registriere Interrupts für die Echo-Pins

        distanceMsgs[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
        distanceMsgs[i].header.frame_id = frameid;
        distanceMsgs[i].field_of_view = 0.26;
        distanceMsgs[i].min_range = 0.02;
        distanceMsgs[i].max_range = 4.0;
        nh.advertise(distancePubs[i]); // Registriere die Publisher
        frameid[6]++;
    }
}

void loop() {
    for (int i = 0; i < numSensors; i++) {
    distanceMsgs[i].range = sonar[i].ping_cm() / 100.0; // Lese die Entfernung in cm und konvertiere sie in Meter
    distancePubs[i].publish(&distanceMsgs[i]); // Veröffentliche die Range-Nachricht
    delay(50);
    }
  
    nh.spinOnce(); // Verarbeite eingehende Nachrichten
    delay(50); // Warte kurz bevor die nächste Messung gestartet wird
}
