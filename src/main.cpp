#include <Arduino.h>
#include "gta_robot.h"
#include "htmlControl.h"

#define TICK_PERIOD 10  //[ms]
hw_timer_t* timer3 = NULL;
volatile bool tick_occurred = false;

GTARobot savage_friday;
const int left_pin = savage_friday.m_l_beacon.getPin();
const int right_pin = savage_friday.m_r_beacon.getPin();

void IRAM_ATTR systemTick_ISR()
{
    tick_occurred = true;
}

/* ISR functions for Beacon Detector */
void IRAM_ATTR beaconLeftDiode_ISR()
{
    // savage_friday.m_l_beacon.processRisingEdge_ISR();
    savage_friday.m_l_beacon.increaseCount();
}

void IRAM_ATTR beaconRightDiode_ISR()
{
    // savage_friday.m_r_beacon.processRisingEdge_ISR();
    savage_friday.m_r_beacon.increaseCount();
}


void checkFreqLeft()
{
    static int old_pin = LOW;
    static uint32_t old_time = micros();

    int current_pin = digitalRead(left_pin);

    if (current_pin == HIGH && old_pin == LOW)
    {
        uint32_t current_time = micros();
        int per = current_time - old_time;
        per = 1000000/per;

        if (per > 700 - 5 && per < 700 + 5)
        {
            // Serial.println("700 Hz");
            savage_friday.m_l_beacon.setFrequency(700);
        }
        else if (per > 23 - 5 && per < 23 + 5)
        {
            // Serial.println("23 Hz");
            savage_friday.m_l_beacon.setFrequency(23);
        }
        else
        {
            // Serial.println("Other");
            savage_friday.m_l_beacon.setFrequency(0);
        }
        old_time = current_time;
    }
    old_pin = current_pin;
}

void checkFreqRight()
{
    static int old_pin = LOW;
    static uint32_t old_time = micros();

    int current_pin = digitalRead(right_pin);

    if (current_pin == HIGH && old_pin == LOW)
    {
        uint32_t current_time = micros();
        int per = current_time - old_time;
        per = 1000000/per;

        if (per > 700 - 5 && per < 700 + 5)
        {
            // Serial.println("700 Hz");
            savage_friday.m_r_beacon.setFrequency(700);
        }
        else if (per > 23 - 5 && per < 23 + 5)
        {
            // Serial.println("23 Hz");
            savage_friday.m_r_beacon.setFrequency(23);
        }
        else
        {
            // Serial.println("Other");
            savage_friday.m_r_beacon.setFrequency(0);
        }
        old_time = current_time;
    }
    old_pin = current_pin;
}

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);

    savage_friday.viveUDPSetup();
    htmlControlInit();

    # if BEACON_USING_INTERRUPT
    // Setup up external interrupts for beacon
    attachInterrupt(digitalPinToInterrupt(savage_friday.m_l_beacon.getPin()),
                    beaconLeftDiode_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(savage_friday.m_r_beacon.getPin()),
                    beaconRightDiode_ISR, RISING);
    #endif

    // Setup tick interrupt function
    timer3 = timerBegin(0, 80, true);       // timer runs at 1MHz and counts up
    timerAttachInterrupt(timer3, systemTick_ISR, true);
    timerAlarmWrite(timer3, (TICK_PERIOD * 1000), true);   // Interrupt occurs every 10ms

    timerAlarmEnable(timer3);
}

void loop()
{
    #if !BEACON_USING_INTERRUPT
    checkFreqLeft();
    checkFreqRight();
    #endif

    if (tick_occurred)
    {
        tick_occurred = false;
        savage_friday.processTick();
    }
}