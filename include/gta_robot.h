#ifndef GTA_ROBOT_H
#define GTA_ROBOT_H

#include "HCSR04.h"
#include "RobotLocomotion.h"

#define ULTRASOIC_SENSOR_NUM 3
#define LATERAL_DIST 10.f // lateral distance should be kept [cm] 
#define FRONT_DIST 18.f // front distance should be kept [cm]


#define LEFT_SENSOR_ECHO_PIN 27
#define FRONT_SENSOR_ECHO_PIN 14
#define RIGHT_SENSOR_ECHO_PIN 33
#define SENSOR_TRIG_PIN 25

class GTARobot
{
public:
    // 2 vive objects

    // 3 ultrasonic sensor objects
    HCSR04 hcsr04 = HCSR04(SENSOR_TRIG_PIN, new int[ULTRASOIC_SENSOR_NUM]{LEFT_SENSOR_ECHO_PIN, FRONT_SENSOR_ECHO_PIN, RIGHT_SENSOR_ECHO_PIN}, ULTRASOIC_SENSOR_NUM); // represent left, front and right in order

    // Motor motor1, motor2; // 2 motor objects
    // Gripper gripper;      // 1 gripper object
    // 1 beacon detector object

    // RobotLocomotion rl = RobotLocomotion(motor1, motor2); // perform basic robot motion
    RobotLocomotion rl; // perform basic robot motion

    GTARobot();
    ~GTARobot();
    void processTick();

    // funtionalilty
    void wallFollowing();
    void beaconSensing();
};

#endif /* GTA_ROBOT */