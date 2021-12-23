#ifndef GTA_ROBOT_H
#define GTA_ROBOT_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HCSR04.h>
#include "vive510.h"
#include "RobotLocomotion.h"
#include "beacon_detector.h"

/* Setup some macros that are used across the different subsystems */

#define SIGNALPIN1 38   // GPIO pin receving signal from Vive circuit
#define SIGNALPIN2 37   // GPIO pin receving signal from Vive circuit

#define BEACON_DIODE_LEFT 36
#define BEACON_DIODE_RIGHT 39
#define BEACON_MAX_PERSISTANCE 5
#define BEACON_TOP_THRESHOLD 3
#define BEACON_BOTTOM_TRESHOLD 1
#define BEACON_JUST_READ_FREQUENCY false

#define DIP_SWITCH_PIN1 13 // dip switch for 1, 2 (ID: 1, 2)
#define DIP_SWITCH_PIN2 15 // dip switch for 3, 4 (ID: 3, 4)

#define LEFT_SENSOR_ECHO_PIN 27
#define FRONT_SENSOR_ECHO_PIN 14
#define RIGHT_SENSOR_ECHO_PIN 33
#define LEFT_SENSOR_TRIG_PIN 25
#define FRONT_SENSOR_TRIG_PIN 26
#define RIGHT_SENSOR_TRIG_PIN 32

// We create two state machines for the robot, one determines the overall state of the system
// and the other determines what actions the robot is performing while in that overall state
enum robot_system_states {MANUAL, AUTONOMOUS, WALL_FOLLOWING, BEACON_SENSING};
enum robot_action_states {IDLE, ADVANCING, TURNING};

// Stuctures to store the incomming UDP robot and can data
typedef struct Robot_data{
    int id;
    int x; 
    int y;
} robot;

typedef struct Can_data{
    int id;
    int x;
    int y;
} can;


class GTARobot
{
public:
    // Have 2 vive objects to compute position and orientation of our robot
    // The actual computation is done in posSending()
    Vive510 vive1 = Vive510(SIGNALPIN1);
    Vive510 vive2 = Vive510(SIGNALPIN2);
    bool pos_valid;     // Indicates whether our computed posistion is valid
    int m_x;            // Robot's x coordinate
    int m_y;            // Robot's y coordinate
    int roboID; 
    char s[13]; // the string to send our robot's position through UDP

    // 3 ultrasonic sensor objects to perdorm wall following
    // We are using a library made by ejoyneering
    HCSR04 left_ultrasonic = HCSR04(LEFT_SENSOR_TRIG_PIN, LEFT_SENSOR_ECHO_PIN, 20, 600);
    HCSR04 front_ultrasonic = HCSR04(FRONT_SENSOR_TRIG_PIN, FRONT_SENSOR_ECHO_PIN, 20, 600);
    HCSR04 right_ultrasonic = HCSR04(RIGHT_SENSOR_TRIG_PIN, RIGHT_SENSOR_ECHO_PIN, 20, 600);

    // Have 2 beacon dectector objects a left and right one to find a beacon on the field
    BeaconDetector m_l_beacon = BeaconDetector(BEACON_DIODE_LEFT);
    BeaconDetector m_r_beacon = BeaconDetector(BEACON_DIODE_RIGHT);

    // Member variables to indicate the state of the robot
    robot_system_states m_system_state;
    robot_action_states m_action_state;

    // Object that operates the motion of the robot
    RobotLocomotion rl = RobotLocomotion();

    // Objects to receive the incoming UDP data from robots and cans
    WiFiUDP robotUDPServer;
    WiFiUDP canUDPServer;
    unsigned int robotUDPPort = 2510;   // port for robot
    unsigned int canUDPPort = 1510;     // port for can

    IPAddress ipTarget = IPAddress(192, 168, 1, 255);
    IPAddress ipLocal1 = IPAddress(192, 168, 1, 132); // my IP address

    const char* ssid = "TP-Link_05AF";
    const char* password = "47543454";
    // const char* ssid = "TP-Link_E0C8";
    // const char* password = "52665134";

    // Arrays to store the actual data from robots and cans
    robot robots[4];
    can cans[8];

public:
    GTARobot();
    ~GTARobot();

    // Function called every 10ms to process our different subsystems:
    // UDP receiver, UDP sender, HTML webpage, Robot locomotion and robot functionality
    void processTick();

    void viveUDPSetup(); // all the settings before using vive and UDP

    // Computes the robot's position as the average of the position calculated by the two vive objects
    void calculateAvgPos(int *, int, int, int, int);
    // The robot's position is filtered to avoid large spikes from erroneous sensor readings
    int calculateMovingAverage(int &value, int newvalue, int weight);
    // Sending the position acquired by two vive through UDP
    void posSending();
    // Helper function to send data through UDP
    void fncUdpSend(char *, int);   

    void handleCanMsg(); // handle received message from can
    void handleRobotMsg(); // handle received message from robot
    void setRoboID();
    void setState(robot_system_states system, robot_action_states action);

    // funtionalilty
    void moveRobot();
    void wallFollowing();
    void beaconSensing();
    void moveToGivenPos();
};

#endif /* GTA_ROBOT */