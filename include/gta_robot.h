#ifndef GTA_ROBOT_H
#define GTA_ROBOT_H

#include <Arduino.h>
#include <HCSR04.h>
#include "RobotLocomotion.h"
#include "vive510.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "beacon_detector.h"

#define SIGNALPIN1 38   // GPIO pin receving signal from Vive circuit
#define SIGNALPIN2 37   // GPIO pin receving signal from Vive circuit

#define BEACON_DIODE_LEFT 36
#define BEACON_DIODE_RIGHT 39
#define BEACON_MAX_PERSISTANCE 5
#define BEACON_TOP_THRESHOLD 2
#define BEACON_BOTTOM_TRESHOLD 1
#define BEACON_USING_INTERRUPT true

#define DIP_SWITCH_PIN1 13 // dip switch for 1, 2 (ID: 1, 2)
#define DIP_SWITCH_PIN2 15 // dip switch for 3, 4 (ID: 3, 4)

#define LEFT_SENSOR_ECHO_PIN 27
#define FRONT_SENSOR_ECHO_PIN 14
#define RIGHT_SENSOR_ECHO_PIN 33
#define LEFT_SENSOR_TRIG_PIN 25
#define FRONT_SENSOR_TRIG_PIN 26
#define RIGHT_SENSOR_TRIG_PIN 32

enum robot_system_states {MANUAL, AUTONOMOUS, WALL_FOLLOWING, BEACON_SENSING};
enum robot_action_states {STOPPED, ADVANCING, TURNING};

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
    // 2 vive objects
    Vive510 vive1 = Vive510(SIGNALPIN1);
    Vive510 vive2 = Vive510(SIGNALPIN2);
    bool pos_valid;
    int m_x;
    int m_y;
    int roboID;
    char s[13]; // the string to send through UDP

    // 3 ultrasonic sensor objects
    HCSR04 left_ultrasonic = HCSR04(LEFT_SENSOR_TRIG_PIN, LEFT_SENSOR_ECHO_PIN, 20, 600);
    HCSR04 front_ultrasonic = HCSR04(FRONT_SENSOR_TRIG_PIN, FRONT_SENSOR_ECHO_PIN, 20, 600);
    HCSR04 right_ultrasonic = HCSR04(RIGHT_SENSOR_TRIG_PIN, RIGHT_SENSOR_ECHO_PIN, 20, 600);

    // Motor motor1, motor2; // 2 motor objects
    // Gripper gripper;      // 1 gripper object

    BeaconDetector m_l_beacon = BeaconDetector(BEACON_DIODE_LEFT);
    BeaconDetector m_r_beacon = BeaconDetector(BEACON_DIODE_RIGHT);

    robot_system_states m_system_state;
    robot_action_states m_action_state;
    RobotLocomotion rl = RobotLocomotion();

    WiFiUDP robotUDPServer;
    WiFiUDP canUDPServer;
    unsigned int robotUDPPort = 2510; // port for robot
    unsigned int canUDPPort = 1510; // port for robot

    IPAddress ipTarget = IPAddress(192, 168, 1, 255);
    IPAddress ipLocal1 = IPAddress(192, 168, 1, 132); // my IP address

    const char* ssid = "TP-Link_05AF";
    const char* password = "47543454";
    // const char* ssid = "TP-Link_E0C8";
    // const char* password = "52665134";

    robot robots[4];
    can cans[8];

public:
    GTARobot();
    ~GTARobot();
    void processTick();

    void viveUDPSetup(); // all the settings before using vive and UDP

    void calculateAvgPos(int *, int, int, int, int); // calculate avg pos from two vive
    int calculateMovingAverage(int &value, int newvalue, int weight);
    void fncUdpSend(char *, int);
    void posSending(); // sending the position acquired by two vive through UDP

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