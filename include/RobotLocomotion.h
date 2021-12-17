#ifndef ROBOT_LOCOMOTION
#define ROBOT_LOCOMOTION

#include <Arduino.h>

#define LIN_VEL_SCALING 0.5 // linear velocity scaling factor
#define ANGLR_VEL_SCALING 0.5 // angular velocity scaling factor

#define TRACK 0.20f // distance between wheels [m]
#define WHEEL_RADIUS 0.035f // radius of the wheel [m]

#define MAX_WHEEL_RPM 170 // to be changed
#define MAX_WHEEL_W (MAX_WHEEL_RPM * 2 * PI / 60)  // [rad/s]
#define MAX_LINEAR_VEL (WHEEL_RADIUS * MAX_WHEEL_W)  // [m/s]
#define MAX_ANGULAR_VEL ((2.0f * WHEEL_RADIUS * MAX_WHEEL_W) / TRACK)  // [rad/s] 

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

enum direction {FORWARD, BACKWARD};

typedef struct robo_vel
{
    float lin_vel; // actual linear velocity of robot [m/s]
    float anglr_vel; // actual angular velocity of robot [rad/s]
} robo_vel;

typedef struct wheel_data
{
    float rad_s; // velocity
    int duty; // duty cycle
    enum direction dir; // direction of rotation
} wheel_data;

class RobotLocomotion
{
public:
    // Motor motor1; // Motor object1
    // Motor motor2; // Motor object2

    robo_vel vel;
    wheel_data l_wheel, r_wheel;

    // constructor & destructor
    // RobotLocomotion(Motor motor1, Motor motor2);
    RobotLocomotion();
    ~RobotLocomotion();

    void turnLeft(float anglr_vel_scal = ANGLR_VEL_SCALING);
    void turnRight(float anglr_vel_scal = ANGLR_VEL_SCALING);
    void goStraight(float lin_vel_scal = LIN_VEL_SCALING);
    void calculate_wheel_vel();
    void updateDirection();
    void updatePWM();
};

#endif