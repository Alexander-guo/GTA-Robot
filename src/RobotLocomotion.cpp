#include "RobotLocomotion.h"

RobotLocomotion::RobotLocomotion(Motor motor1, Motor motor2){
    this -> motor1 = motor1;
    this -> motor2 = motor2;
    robo_vel vel{MAX_LINEAR_VEL * LIN_VEL_SCALING, 0};
}

void RobotLocomotion::turnLeft(float anglr_vel_scal = ANGLR_VEL_SCALING){
    vel.lin_vel = MAX_LINEAR_VEL * LIN_VEL_SCALING; // set linear velocity back to initial value
    vel.anglr_vel = MAX_ANGULAR_VEL * anglr_vel_scal;
    calculate_wheel_vel();
    updateDirection();
    updatePWM();
}

void RobotLocomotion::turnRight(float anglr_vel_scal = ANGLR_VEL_SCALING){
    vel.lin_vel = MAX_LINEAR_VEL * LIN_VEL_SCALING; // set linear velocity back to initial value 
    vel.anglr_vel = -MAX_ANGULAR_VEL * anglr_vel_scal;
    calculate_wheel_vel();
    updateDirection();
    updatePWM(); 
}

void RobotLocomotion::goStraight(float lin_vel_scal = LIN_VEL_SCALING){
    vel.lin_vel = MAX_LINEAR_VEL * lin_vel_scal;
    vel.anglr_vel = 0;
    calculate_wheel_vel();
    updateDirection();
    updatePWM();
}

void RobotLocomotion::calculate_wheel_vel(){
    // compute the velocity of each wheel in [rad/s]
    r_wheel.rad_s = vel.lin_vel + (vel.anglr_vel * TRACK)/2.0f;	// linear velocity for wheel m/s
    l_wheel.rad_s = vel.lin_vel - (vel.anglr_vel * TRACK)/2.0f;	// linear velocity for wheel m/s

    r_wheel.rad_s /= WHEEL_RADIUS;	//angular velocity rad/s
    l_wheel.rad_s /= WHEEL_RADIUS;	//angular velocity rad/s

    // judge the wheel direction
    r_wheel.dir = r_wheel.rad_s < 0 ? BACKWARD : FORWARD;
    l_wheel.dir = l_wheel.rad_s < 0 ? BACKWARD : FORWARD;

    // calculate the duty cycle
    r_wheel.duty = 100.0f * abs(r_wheel.rad_s) / MAX_WHEEL_W;
    l_wheel.duty = 100.0f * abs(l_wheel.rad_s) / MAX_WHEEL_W;

    // Clip the duty cycle for safety
    r_wheel.duty = max(0, min(100, r_wheel.duty));
    l_wheel.duty = max(0, min(100, l_wheel.duty));
}

void RobotLocomotion::updateDirection(){
    if (r_wheel.dir == FORWARD)
    {
        digitalWrite(MOTOR_R_DIR_PIN, HIGH);
        digitalWrite(MOTOR_R_NDIR_PIN, LOW);
    }
    else
    {
        digitalWrite(MOTOR_R_DIR_PIN, LOW);
        digitalWrite(MOTOR_R_NDIR_PIN, HIGH);
    }

    if (l_wheel.dir == FORWARD)
    {
        digitalWrite(MOTOR_L_DIR_PIN, LOW);
        digitalWrite(MOTOR_L_NDIR_PIN, HIGH);
    }
    else
    {
        digitalWrite(MOTOR_L_DIR_PIN, HIGH);
        digitalWrite(MOTOR_L_NDIR_PIN, LOW);
    }
}

void RobotLocomotion::updatePWM(){
    // Compute proportional duty cycles based on percentage duty cycles
    int r_wheel_prop_duty = map(r_wheel.duty, 0, 100, 0, LEDC_RESOLUTION);
    int l_wheel_prop_duty = map(l_wheel.duty, 0, 100, 0, LEDC_RESOLUTION);

    ledcWrite(LEDC_R_CHANNEL, r_wheel_prop_duty);     // Update the Duty Cycle
    ledcWrite(LEDC_L_CHANNEL, l_wheel_prop_duty);     // Update the Duty Cycle
}