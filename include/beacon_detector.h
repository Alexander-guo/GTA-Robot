#ifndef BEACON_DETECTOR_H
#define BEACON_DETECTOR

#define MAX_FREQ 700
#define MIN_FREQ 23
#define NOISE_FREQ 5

#include <Arduino.h>

class BeaconDetector
{
private:
    int m_pin;
    int m_frequency;
    uint32_t m_counts;
    uint32_t m_t_now;
    uint32_t m_t_prev;
    portMUX_TYPE m_mux = portMUX_INITIALIZER_UNLOCKED;
    bool m_detected_a_pulse;
    int m_ticks_since_pulse;
public:
    BeaconDetector();
    BeaconDetector(int pin);

    int getPin();
    int getFrequency();
    void setFrequency(int freq);
    // Computes the frequency of the signal given the time between each rising edge
    void IRAM_ATTR processRisingEdge_ISR();
    // Call every 10ms to verify that there is still a signal
    void verifyFrequency();
};

#endif /* BEACON_DETECTOR_H */