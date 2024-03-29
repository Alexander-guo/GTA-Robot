#include "beacon_detector.h"

BeaconDetector::BeaconDetector()
    : BeaconDetector(0)
{}

BeaconDetector::BeaconDetector(int pin)
    : m_pin(pin)
{
    pinMode(m_pin, INPUT);
}


int BeaconDetector::getPin()
{
    return m_pin;
}

int BeaconDetector::getFrequency()
{
    portENTER_CRITICAL_ISR(&m_mux);
    float freq = m_frequency;
    portEXIT_CRITICAL_ISR(&m_mux);
    return freq;
}

void BeaconDetector::setFrequency(int freq)
{
    portENTER_CRITICAL_ISR(&m_mux);
    m_frequency = freq;
    portEXIT_CRITICAL_ISR(&m_mux);
}

// Computes the frequency of the signal given the time between each rising edge
void IRAM_ATTR BeaconDetector::processRisingEdge_ISR()
{
    m_detected_a_pulse = true;
    m_ticks_since_pulse = 0;
    m_t_now = micros();

    int new_value = 1000000ULL / (m_t_now - m_t_prev);

    if (new_value > MAX_FREQ - NOISE_FREQ && new_value < MAX_FREQ + NOISE_FREQ)
    {
        setFrequency(700);
    }
    else if (new_value > MIN_FREQ - NOISE_FREQ && new_value < MIN_FREQ + NOISE_FREQ)
    {
        setFrequency(23);
    }
    else
    {
        setFrequency(new_value);
    }
    m_t_prev = m_t_now;
}

/*  Call every 10ms to verify that there is still a signal.
*   Since we know what the lowest signal frequency we want to detect is, we
*   can set the frequency to 0, if the amount of time elapsed since the last
*   rising edge interrupt ocurred is larger than the period of our signal
*/
void BeaconDetector::verifyFrequency()
{
    m_ticks_since_pulse++;

    if (!m_detected_a_pulse)
    {
        if (m_ticks_since_pulse > 5)
        {
            setFrequency(0);
        }
        else
        {
            // Otherwise don't update the frequency
            return;
        }
    }
    else
    {
        m_detected_a_pulse = false;
    }
}