/*
 * Gripper Functionality
 *
 * Dec. 7, 2021
 */

#include "gripper.h"

Gripper::Gripper(){

}

Gripper::Gripper(int pin, uint8_t channel, int res_bits){
    ledc_pin_servo = pin;
    ledc_channel = channel;
    ledc_res_bits = res_bits;
}

void Gripper::setup(){
    // set up ledc
    ledcSetup(ledc_channel, LEDC_FREQ_HZ, ledc_res_bits);
    ledcAttachPin(ledc_pin_servo, ledc_channel);
    ledcWrite(ledc_channel, 0);
}

void Gripper::ledcAnalogWrite(uint8_t channel, float duty_ratio_ms, float duty_max = LEDC_PULSE_PERIOD){
    uint32_t duty_res = ledc_res * duty_ratio_ms / LEDC_PULSE_PERIOD;
    Serial.printf("%f,  %f,  %d\n", duty_ratio_ms, duty_ratio_ms / LEDC_PULSE_PERIOD * 100, duty_res);
    ledcWrite(channel, duty_res);
}

void Gripper::gripperOpen(){
    // open the gripper
    for(float duty_ratio_ms = duty_ratio_ms_limit; duty_ratio_ms >= DUTY_RATIO_MIN - 0.1;)
    {
        ledcAnalogWrite(ledc_channel, duty_ratio_ms);
        duty_ratio_ms -= 0.1;
        delay(100);
    }
}

void Gripper::gripperClose(){
    // close the gripper
    for(float duty_ratio_ms = DUTY_RATIO_MIN; duty_ratio_ms <= duty_ratio_ms_limit + 0.1;)
    {
        ledcAnalogWrite(ledc_channel, duty_ratio_ms);
        duty_ratio_ms += 0.1;
        delay(100);
    }
}




