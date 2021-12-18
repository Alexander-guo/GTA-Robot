#ifndef GTA_ROBOT_H
#define GTA_ROBOT_H

#include <Arduino.h>
#include <HCSR04.h>
#include "RobotLocomotion.h"
#include "vive510.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define SIGNALPIN1 38   // GPIO pin receving signal from Vive circuit
#define SIGNALPIN2 37   // GPIO pin receving signal from Vive circuit

#define DIP_SWITCH_PIN1 13 // dip switch for 1, 2 (ID: 1, 2)
#define DIP_SWITCH_PIN2 15 // dip switch for 3, 4 (ID: 3, 4)


#define ULTRASOIC_SENSOR_NUM 3
#define LATERAL_DIST 15.f // lateral distance should be kept [cm] 
#define FRONT_DIST 24.f // front distance should be kept [cm]


#define LEFT_SENSOR_ECHO_PIN 27
#define FRONT_SENSOR_ECHO_PIN 14
#define RIGHT_SENSOR_ECHO_PIN 33
#define LEFT_SENSOR_TRIG_PIN 25
#define FRONT_SENSOR_TRIG_PIN 26
#define RIGHT_SENSOR_TRIG_PIN 32
// #define SENSOR_TRIG_PIN 25

enum robot_states {MANUAL, AUTONOMOUS, WALL_FOLLOWING, BEACON_SENSING};

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
    // 3 ultrasonic sensor objects
    // HCSR04 hcsr04 = HCSR04(SENSOR_TRIG_PIN, new int[ULTRASOIC_SENSOR_NUM]{LEFT_SENSOR_ECHO_PIN, FRONT_SENSOR_ECHO_PIN, RIGHT_SENSOR_ECHO_PIN}, ULTRASOIC_SENSOR_NUM); // represent left, front and right in order
    HCSR04 left_ultrasonic = HCSR04(LEFT_SENSOR_TRIG_PIN, LEFT_SENSOR_ECHO_PIN, 20, 600);
    HCSR04 front_ultrasonic = HCSR04(FRONT_SENSOR_TRIG_PIN, FRONT_SENSOR_ECHO_PIN, 20, 600);
    HCSR04 right_ultrasonic = HCSR04(RIGHT_SENSOR_TRIG_PIN, RIGHT_SENSOR_ECHO_PIN, 20, 600);

    // Motor motor1, motor2; // 2 motor objects
    // Gripper gripper;      // 1 gripper object
    // 1 beacon detector object
    RobotLocomotion rl = RobotLocomotion();

    WiFiUDP robotUDPServer;
    WiFiUDP canUDPServer;
    unsigned int robotUDPPort = 2510; // port for robot
    unsigned int canUDPPort = 1510; // port for robot

    IPAddress ipTarget = IPAddress(192, 168, 0, 255);
    IPAddress ipLocal1 = IPAddress(192, 168, 0, 132); // my IP address
    const char* ssid = "TP-Link_05AF";
    const char* password = "47543454";

    robot robots[4];
    can cans[8];

    char s[13]; // the string to send through UDP
    int roboID;

    robot_states m_robo_state;
    // RobotLocomotion rl = RobotLocomotion(motor1, motor2); // perform basic robot motion

    GTARobot();
    ~GTARobot();
    void processTick();

    void viveUDPSetup(); // all the settings before using vive and UDP

    void calculateAvgPos(int *, int, int, int, int); // calculate avg pos from two vive
    void fncUdpSend(char *, int);
    void posSending(); // sending the position acquired by two vive through UDP

    void handleCanMsg(); // handle received message from can
    void handleRobotMsg(); // handle received message from robot
    void setRoboID();
    void setState(robot_states state);

    // funtionalilty
    void moveRobot();
    void wallFollowing();
    void beaconSensing();
    void moveToGivenPos();
};

#endif /* GTA_ROBOT */