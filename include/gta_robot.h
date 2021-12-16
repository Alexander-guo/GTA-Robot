#ifndef GTA_ROBOT_H
#define GTA_ROBOT_H

#include "HCSR04.h"
#include "RobotLocomotion.h"

#define ULTRASOIC_SENSOR_NUM 3
#define LATERAL_DIST 10.f // lateral distance should be kept [cm] 
#define FRONT_DIST 18.f // front distance should be kept [cm]

#define TRACK 0.20f // distance between wheels [m]
#define WHEEL_RADIUS 0.035f // radius of the wheel [m]

#define MAX_WHEEL_RPM 170 // to be changed
#define MAX_WHEEL_W (MAX_WHEEL_RPM * 2 *  PI / 60)  // [rad/s]
#define MAX_LINEAR_VEL (WHEEL_RADIUS * MAX_WHEEL_W)  // [m/s]
#define MAX_ANGULAR_VEL ((2.0f * WHEEL_RADIUS * MAX_WHEEL_W) / TRACK)  // [rad/s] 
#define LIN_VEL_SCALING 0.5 // linear velocity scaling factor
#define ANGLR_VEL_SCALING 0.5 // angular velocity scaling factor

#define MOTOR_L_PWM_PIN 5 // ENA
#define MOTOR_L_DIR_PIN 18 // IN1
#define MOTOR_L_NDIR_PIN 23 // IN2
#define MOTOR_R_PWM_PIN 21 // ENB
#define MOTOR_R_DIR_PIN 19 // IN3
#define MOTOR_R_NDIR_PIN 22 // IN4

#define LEDC_L_CHANNEL 0
#define LEDC_R_CHANNEL 1
#define LEDC_RESOLUTION_BITS 13 
#define LEDC_RESOLUTION ((1<<LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 50

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

    Motor motor1, motor2; // 2 motor objects
    Gripper gripper;      // 1 gripper object
    // 1 beacon detector object

    RobotLocomotion rl = RobotLocomotion(motor1, motor2); // perform basic robot motion

    GTARobot();
    ~GTARobot();
    void processTick();

    // funtionalilty
    void wallFollowing();
    void beaconSensing();
};

#endif /* GTA_ROBOT */