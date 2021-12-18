#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class Ultrasonic
{
private:
    int m_trigger_pin;
    int m_echo_pin;
    float distance;
public:
    Ultrasonic();
    Ultrasonic(int trig_pin, int echo_pin);
};

#endif /* ULTRASONIC_SENSOR_H */