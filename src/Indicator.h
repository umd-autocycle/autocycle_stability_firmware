//
// Created by Misha on 6/25/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_INDICATOR_H
#define AUTOCYCLE_STABILITY_FIRMWARE_INDICATOR_H

#include <Arduino.h>

#define PWM_RES     10U
#define PWM_MAX     4095
#define F_BEEP      1320
#define F_BOOP      524


class Indicator {
public:
    Indicator(uint8_t r_pin, uint8_t g_pin, uint8_t b_pin, uint8_t buzz_pin, uint8_t loud_buzz_pin);

    void start();

    void setPassiveRGB(uint8_t r, uint8_t g, uint8_t b);
    void setBlinkRGB(uint8_t r, uint8_t g, uint8_t b);

    void cycle();
    void update();

    void beep(unsigned int duration = 500) const; // beep and boop do not block until end of tone, return after setting
    void boop(unsigned int duration = 500) const;
    void silence() const;

    void setPulse(unsigned int duration, unsigned int interval);
    void disablePulse();

    void beepstring(uint8_t bitstring, int bitrate = 2);
    void beepstring(uint16_t bitstring, int bitrate = 2);
    void beepstring(uint32_t bitstring, int bitrate = 2);
    void beepstring(uint64_t bitstring, int bitrate = 2);

    void yell(unsigned int duration);

private:
    uint8_t r_pin, g_pin, b_pin, buzz_pin, loud_buzz_pin;
    uint16_t pr, pg, pb, br, bg, bb;
    bool pulse_on, pulsed, yelling;
    unsigned int pulse_duration, pulse_interval, yell_duration;
    unsigned long pulse_ref, yell_start;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_INDICATOR_H
