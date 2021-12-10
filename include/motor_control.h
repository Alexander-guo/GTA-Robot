#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "helper_tools.h"
enum direction {BACKWARD, FORWARD};

class Motor
{
public:
    int m_ledc_chan;
    int m_out_pin;
    int m_dir_pin;
    int m_dir_not_pin;
    int m_enca_pin;
    int m_encb_pin;
    enum direction m_direction;
    float m_rpm;
    int m_ticks_since_enc_trig;
    uint64_t m_enc_counts;
    uint64_t m_dt;
public:
    float m_kp, m_ki, m_kd;
    float m_target;

    Motor(int ledc_chan, int out_pin, int dir_pin, int dir_not_pin, int enca_pin, int encb_pin);
    ~Motor();

    void setPins(int ledc_chan, int dir, int dir_not, int enca, int encb);
    void setDirection(direction dir);
    void setDutyCycle(float duty);
    void setRPM(float speed);
    void handleEncoderInterrupt(bool enca_triggerd);
    void computeRPM();

    float getRPM() const;
    direction getDirection() const;
};


#endif  // MOTOR_CONTROL_H