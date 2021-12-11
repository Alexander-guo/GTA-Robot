/*
 * Header for Gripper
 *
 * Dec. 7, 2021 
 */

#ifndef GRIPPER
#define GRIPPER

#include <Arduino.h>


#define LEDC_FREQ_HZ 50
#define LEDC_PULSE_PERIOD (1000.f/LEDC_FREQ_HZ)  // pulse perid in [ms]
#define DUTY_RATIO_MIN 0.5 // [ms] 0 degree
#define DUTY_RATIO_MAX 2.5 // [ms] 180 degree 


class Gripper
{
private:
    const double duty_ratio_ms_limit = 1.3; // max duty ratio for the gripper to close in [ms]
    uint8_t ledc_channel = 0;
    int ledc_res_bits = 13;
    int ledc_pin_servo = 15;
    uint32_t ledc_res = ((1<<ledc_res_bits) - 1); 

public:
    Gripper(int, uint8_t, int);
    Gripper();
    void setup();
    void gripperOpen();
    void gripperClose();
    void ledcAnalogWrite(uint8_t, float, float);
};

#endif