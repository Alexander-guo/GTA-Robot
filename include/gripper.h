/*
 * Header for Gripper
 *
 * Dec. 7, 2021 
 */

#ifndef GRIPPER
#define GRIPPER

#include <Arduino.h>

#define TOUCH_PIN 32
#define LEDC_FREQ_HZ 50
#define LEDC_PULSE_PERIOD (1000.f/LEDC_FREQ_HZ)  // pulse perid in [ms]
#define DUTY_RATIO_MIN 0.5 // [ms] 0 degree
#define DUTY_RATIO_MAX 2.5 // [ms] 180 degree 


class Gripper
{
    private:
        static const float duty_ratio_ms_limit; // max duty ratio for the gripper to close in [ms]
        uint8_t ledc_channel = 0;
        int ledc_res_bits = 13;
        int ledc_pin_servo = 26;
        uint32_t ledc_res = ((1 << ledc_res_bits) - 1);

    public:
        Gripper(int, uint8_t, int);
        Gripper();
        void setup();
        void gripperOpen(float x = Gripper::duty_ratio_ms_limit);
        float gripperClose();
        void ledcAnalogWrite(uint8_t, float, float);
        boolean isGripped();
};

#endif