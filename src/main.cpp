#include <Arduino.h>
#include "gta_robot.h"

#define TICK_PERIOD 10  //[ms]
hw_timer_t* timer3 = NULL;
volatile bool tick_occurred = false;

GTARobot savage_friday;

void IRAM_ATTR systemTick_ISR()
{
    tick_occurred = true;
}

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);

    savage_friday.viveUDPSetup();

    // Setup tick interrupt function
    timer3 = timerBegin(0, 80, true);       // timer runs at 1MHz and counts up
    timerAttachInterrupt(timer3, systemTick_ISR, true);
    timerAlarmWrite(timer3, (TICK_PERIOD * 1000), true);   // Interrupt occurs every 10ms

    timerAlarmEnable(timer3);
}

void loop()
{
    if (tick_occurred)
    {
        tick_occurred = false;
        savage_friday.processTick();
    }
}