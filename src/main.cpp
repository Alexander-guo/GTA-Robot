#include <Arduino.h>
#include "motor_control.h"
#include "helper_tools.h"
#include "Plotter.h"
#include <PID_v1.h>

#define ENABLE_PLOTTING true
#define SAMPLE_TIME 10  //[ms]

#define LEDC_R_CHANNEL 0
#define MOTOR_R_ENCA 21         // Connect to motor YELLOW wire
#define MOTOR_R_ENCB 22         // Connect to motor WHITE wire
#define MOTOR_R_OUT 19          // Connect to ENB on driver
#define MOTOR_R_DIR_PIN 23      // Connect to IN4 on driver
#define MOTOR_R_NOT_DIR_PIN 18  // Connect to IN3 on driver

hw_timer_t* timer3 = NULL;
bool tick_flag = false;

Motor r_wheel(LEDC_R_CHANNEL, MOTOR_R_OUT, MOTOR_R_DIR_PIN,
              MOTOR_R_NOT_DIR_PIN, MOTOR_R_ENCA, MOTOR_R_ENCB);

Plotter p;

double Setpoint, Input, Output;
double Kp=2, Ki=5.0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Tick interrupt function
void IRAM_ATTR systemTick_ISR()
{
    tick_flag = true;
}

// Interrupt Service Routines
void IRAM_ATTR rightMotorEncoderA_ISR()
{
    r_wheel.handleEncoderInterrupt(true);
}

void IRAM_ATTR rightMotorEncoderB_ISR()
{
    r_wheel.handleEncoderInterrupt(false);
}

void setup()
{
    Serial.begin(115200);

    // Setup a button
    pinMode(25, INPUT_PULLUP);

    // Setup Tick interrupt function
    timer3 = timerBegin(0, 80, true);   // timer runs at 1MHz and counts up
    timerAttachInterrupt(timer3, systemTick_ISR, true);
    timerAlarmWrite(timer3, 10000, true);   // Interrupt occurs every 10ms

    // Default initial motor state
    r_wheel.setRPM(0);
    attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENCA), rightMotorEncoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_R_ENCB), rightMotorEncoderB_ISR, CHANGE);

    // Initialize PID
    Input = r_wheel.getRPM();
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(SAMPLE_TIME);
    myPID.SetOutputLimits(-170, 170);

#if ENABLE_PLOTTING
    // Setup the plotter
    p.Begin();
    p.AddTimeGraph("PID Control", 1000, "Input", Input, "Output", Output, "Setpoint", Setpoint);
#endif

    // Wait for 3 seconds before starting the program
    for (int i = 0; i < 3; i++)
    {
        delay(1000);
    }
    timerAlarmEnable(timer3);
}

void loop()
{
    if (tick_flag)
    {
        tick_flag = false;
        if (digitalRead(25) == HIGH)
        {
            Setpoint = 0;
        }
        else
        {
            Setpoint = 100;
        }

        r_wheel.computeRPM();
        Input = r_wheel.getRPM();
        myPID.Compute();
        r_wheel.setRPM(Output);

    #if ENABLE_PLOTTING
        p.Plot();
    #else
        Serial.println(r_wheel.getRPM());
    #endif

    }
}
