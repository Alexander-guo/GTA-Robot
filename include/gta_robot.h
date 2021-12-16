#ifndef GTA_ROBOT_H
#define GTA_ROBOT_H

#include <Arduino.h>
#include "vive510.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define SIGNALPIN1 38 // GPIO pin receving signal from Vive circuit

/* comment it out if you only use one vive circuit */
#define SIGNALPIN2 37

#define ID_12_PIN 13 // dip switch for 1, 2 (ID: 1, 2)
#define ID_34_PIN 15 // dip switch for 3, 4 (ID: 3, 4)


class GTARobot
{
public:
    // 2 vive objects
    Vive510 vive1 = Vive510(SIGNALPIN1);
    Vive510 vive2 = Vive510(SIGNALPIN2);
    // 3 ultrasonic sensor objects
    // 2 motor objects
    // 1 gripper object
    // 1 beacon detector object

    WiFiUDP robotUDPServer;
    WiFiUDP canUDPServer;
    unsigned int robotUDPPort = 2510; // port for robot
    unsigned int canUDPPort = 1510; // port for robot

    IPAddress ipTarget = IPAddress(192, 168, 0, 255);
    IPAddress ipLocal1 = IPAddress(192, 168, 0, 132); // my IP address
    const char* ssid = "TP-Link_05AF";
    const char* password = "47543454";

    char s[13]; // the string to send through UDP
    int roboID;

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

    robot* robots; // position data for all robots on the field
    can* cans; // position data for all cans on the field


    GTARobot();
    ~GTARobot();
    void processTick();

    void viveUDPSetup(); // all the settings before using vive and UDP

    void calculateAvgPos(int *, int, int, int, int); // calculate avg pos from two vive
    void fncUdpSend(char *, int);
    void posSending(); // sending the position acquired by two vive through UDP

    void handleCanMsg(); // handle received message from can
    void handleRoboMsg(); // handle received message from robot
    void setRoboID();
};

#endif /* GTA_ROBOT */