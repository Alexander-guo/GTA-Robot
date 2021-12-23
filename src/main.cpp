#include <Arduino.h>
#include "gta_robot.h"
#include "htmlControl.h"

#define TICK_PERIOD 10  //[ms]
hw_timer_t* timer3 = NULL;
volatile bool tick_occurred = false;

GTARobot savage_friday;     // This object represents are whole robot which has many subsystems

void IRAM_ATTR systemTick_ISR()
{
    tick_occurred = true;
}

/* ISR functions for Beacon Detector */
void IRAM_ATTR beaconLeftDiode_ISR()
{
    savage_friday.m_l_beacon.processRisingEdge_ISR();
}

void IRAM_ATTR beaconRightDiode_ISR()
{
    savage_friday.m_r_beacon.processRisingEdge_ISR();
}

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);

    savage_friday.viveUDPSetup();
    htmlControlInit();

    // Setup up external interrupts for beacon
    attachInterrupt(digitalPinToInterrupt(savage_friday.m_l_beacon.getPin()),
                    beaconLeftDiode_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(savage_friday.m_r_beacon.getPin()),
                    beaconRightDiode_ISR, RISING);

    // Setup tick interrupt function
    timer3 = timerBegin(0, 80, true);       // timer runs at 1MHz and counts up
    timerAttachInterrupt(timer3, systemTick_ISR, true);
    timerAlarmWrite(timer3, (TICK_PERIOD * 1000), true);   // Interrupt occurs every 10ms

    timerAlarmEnable(timer3);
}

void loop()
{
    // We process all of our subsystems every 10ms. In an interrupt we just set
    // a flag to indicate that it is time to processa a tick, but the actual
    // processing occurs in the main loop.
    if (tick_occurred)
    {
        tick_occurred = false;
        savage_friday.processTick();
    }
}