#ifndef BEACON_DETECTOR_H
#define BEACON_DETECTOR

#define MAX_FREQ 700
#define MIN_FREQ 23
#define NOISE_FREQ 5

#include <Arduino.h>

class BeaconDetector
{
public:
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
    void IRAM_ATTR processRisingEdge_ISR();
    void increaseCount();
    void computeFrequency();
};

#endif /* BEACON_DETECTOR_H */