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

void IRAM_ATTR BeaconDetector::processRisingEdge_ISR()
{
    // if (!m_detected_a_pulse)
    // {
    //     return;
    // }
    // m_detected_a_pulse = false;
    // m_t_now = millis();
    // float new_value = 1000ULL * m_counts / (m_t_now - m_t_prev);
    // m_counts = 0;

    // if (new_value > MAX_FREQ - NOISE_FREQ && new_value < MAX_FREQ + NOISE_FREQ)
    // {
    //     m_frequency = 700;
    // }
    // else if (new_value > MIN_FREQ - NOISE_FREQ && new_value < MIN_FREQ + NOISE_FREQ)
    // {
    //     m_frequency = 23;
    // }
    // else
    // {
    //     m_frequency = new_value;
    // }
    // m_t_prev = m_t_now;

    // m_t_now = micros();
    int new_value = 1000000ULL / (m_t_now - m_t_prev);

    if (new_value > MAX_FREQ - NOISE_FREQ && new_value < MAX_FREQ + NOISE_FREQ)
    {
        m_frequency = 700;
    }
    else if (new_value > MIN_FREQ - NOISE_FREQ && new_value < MIN_FREQ + NOISE_FREQ)
    {
        m_frequency = 23;
    }
    else
    {
        m_frequency = new_value;
    }

    m_t_prev = m_t_now;
}

void BeaconDetector::increaseCount()
{
    portENTER_CRITICAL_ISR(&m_mux);
    m_counts++;
    m_detected_a_pulse = true;
    m_ticks_since_pulse = 0;
    m_t_prev = m_t_now;
    m_t_now = micros();
    portEXIT_CRITICAL_ISR(&m_mux);
}

void BeaconDetector::computeFrequency()
{
    m_ticks_since_pulse++;

    if (m_detected_a_pulse == false)
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
        int new_value = m_t_now - m_t_prev;
        if (new_value != 0)
        {
            new_value = 1000000 / new_value;
        }

        if (new_value > MAX_FREQ - 10 && new_value < MAX_FREQ + 10)
        {
            setFrequency(700);
        }
        else if (new_value > MIN_FREQ - 3 && new_value < MIN_FREQ + 3)
        {
            setFrequency(23);
        }
        else
        {
            setFrequency(0);
        }
    }
}