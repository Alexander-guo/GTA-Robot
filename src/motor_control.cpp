#include <Arduino.h>
#include "motor_control.h"

#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)    // The maximum number stored in the ADC register
#define LEDC_FREQ_HZ 50

#define MAX_WHEEL_W 170.0f // RPM no load
#define COUNTS_PER_REVOLUTION 2248.8576    // This is taking into account only the rising edge of one of the encoder channels

Motor::Motor(int ledc_chan, int out_pin, int dir_pin, int dir_not_pin, int enca_pin, int encb_pin)
    :m_ledc_chan(ledc_chan), m_out_pin(out_pin), m_dir_pin(dir_pin), m_dir_not_pin(dir_not_pin),
    m_enca_pin(enca_pin), m_encb_pin(encb_pin)
{
    // Configure the ledc channel to operate the pwm output for the motor
    ledcSetup(this->m_ledc_chan, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    ledcAttachPin(this->m_out_pin, this->m_ledc_chan);  // This function also sets the pin mode

    // Set the pin modes
    pinMode(this->m_dir_pin, OUTPUT);
    pinMode(this->m_dir_not_pin, OUTPUT);
    pinMode(this->m_enca_pin, INPUT);
    pinMode(this->m_encb_pin, INPUT);
}

Motor::~Motor()
{
    ledcDetachPin(this->m_out_pin);
}

void Motor::setDirection(direction dir)
{
    // Change the GPIO pin output states connected to the half bridge
    // depending on the direction that the motor shoudl be spinning
    if (dir == FORWARD)
    {
        digitalWrite(this->m_dir_pin, HIGH);
        digitalWrite(this->m_dir_not_pin, LOW);
    }
    else if (dir == BACKWARD)
    {
        digitalWrite(this->m_dir_pin, LOW);
        digitalWrite(this->m_dir_not_pin, HIGH);
    }
}

void Motor::setDutyCycle(float duty)
{
    // Compute proportional duty cycles based on percentage duty cycles
    int motor_prop_duty = int(mapFloat(duty, 0.0f, 100.0f, 0.0f, LEDC_RESOLUTION));

    ledcWrite(this->m_ledc_chan, motor_prop_duty);     // Update the Duty Cycle
}

// This function takes in values in the range [-170,170]
void Motor::setRPM(float angular_velocity)
{
    float speed = fabs(angular_velocity);       // Get the magnitude
    speed = min(MAX_WHEEL_W, max(speed, 0.0f)); // Clip the value to [0, 170]

    direction dir = angular_velocity < 0 ? BACKWARD : FORWARD;
    float motor_duty = 100.0f * speed / MAX_WHEEL_W;  // Compute duty cycle based on setpoint and max angular velocity

    setDirection(dir);
    setDutyCycle(motor_duty);
}

void IRAM_ATTR Motor::handleEncoderInterrupt(bool enca_triggered)
{
    // Add to count for both rising and falling edge of both channel A and B
    static uint64_t prev_time;
    static uint64_t counter;

    m_ticks_since_enc_trig = 0;
    m_enc_counts++;

    if (++counter == 16)
    {
        counter = 0;
        m_enc_counts = 0;
        m_dt = micros() - prev_time;
        prev_time = micros();
    }

    // Only determine motor direction on rising edge of channel A
    if (enca_triggered && digitalRead(this->m_enca_pin) == HIGH)
    {
        if (digitalRead(this->m_encb_pin) == HIGH)
        {
            this->m_direction = FORWARD;
        }
        else
        {
            this->m_direction = BACKWARD;
        }
    }
}

void Motor::computeRPM()
{
    /* Our tick function runs every 10ms and the slowest non-zero speed
    *   that we have decided a motor can be is 1 RPM. This means that if
    *   there isn't at least 1 count in 3 ticks (actually 26.68ms) then the
    *   motor is halted. */

   if (++m_ticks_since_enc_trig >= 3)
   {
        // this->m_rpm = double(m_enc_counts * 6000ULL) / double(COUNTS_PER_REVOLUTION * m_ticks_since_enc_trig);
        m_enc_counts = 0;
        this->m_rpm = 0;
   }
   else
   {
       if (m_dt != 0)
       {
            this->m_rpm = double(16ULL * 60000000ULL) / double(COUNTS_PER_REVOLUTION * m_dt);
       }
   }


    if (getDirection() == BACKWARD)
    {
        this->m_rpm *= -1;
    }
}

// Getters
float Motor::getRPM() const
{
    return this->m_rpm;
}

direction Motor::getDirection() const
{
    return this->m_direction;
}