#ifndef ROBOT_LOCOMOTION
#define ROBOT_LOCOMOTION

#include "gta_robot.h"

class RobotLocomotion
{
public:
    Motor motor1; // Motor object1
    Motor motor2; // Motor object2

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

    robo_vel vel;
    wheel_data l_wheel, r_wheel;

    // constructor & destructor
    RobotLocomotion(Motor motor1, Motor motor2);
    ~RobotLocomotion();

    void turnLeft(float anglr_vel_scal = ANGLR_VEL_SCALING);
    void turnRight(float anglr_vel_scal = ANGLR_VEL_SCALING);
    void goStraight(float lin_vel_scal = LIN_VEL_SCALING);
    void calculate_wheel_vel();
    void updateDirection();
    void updatePWM();
};

#endif