/*
 * Gripper Functionality
 *
 * Dec. 7, 2021
 */

#include "gripper.h"

const float Gripper::duty_ratio_ms_limit = 1.3;

Gripper::Gripper()
{
    pinMode(TOUCH_PIN, INPUT);
}

Gripper::Gripper(int pin, uint8_t channel, int res_bits){
    pinMode(TOUCH_PIN, INPUT);
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

void Gripper::gripperOpen(float start_duty_ratio_ms){
    // open the gripper
    static uint64_t previous_time;

    for(float duty_ratio_ms = start_duty_ratio_ms; duty_ratio_ms >= DUTY_RATIO_MIN - 0.1;)
    {
        if(millis() - previous_time >= 100){
            ledcAnalogWrite(ledc_channel, duty_ratio_ms);
            duty_ratio_ms -= 0.1;
            previous_time = millis();
        }
        // delay(100);
    }
}

float Gripper::gripperClose(){
    // close the gripper
    static uint64_t previous_time;
    float duty_ratio_ms;
    for (duty_ratio_ms = DUTY_RATIO_MIN; duty_ratio_ms <= duty_ratio_ms_limit + 0.1;)
    {
        if (millis() - previous_time >= 100){
            ledcAnalogWrite(ledc_channel, duty_ratio_ms);
            duty_ratio_ms += 0.1;
            previous_time = millis();
            if (isGripped()){
                Serial.println("Yes!!!!");
                break;
            }
            else{
                Serial.println("No!!!!");
            }
        }
    }
    return duty_ratio_ms;
    // delay(100);
}

boolean Gripper::isGripped(){
    if(touchRead(TOUCH_PIN) <= 5){
        return true;
    }
    else{
        return false;
    }
}




