//
// Created by Misha on 6/25/2020.
//

#include <Arduino.h>
#include "Tone.h"

#include "Indicator.h"

Indicator::Indicator(uint8_t r_pin, uint8_t g_pin, uint8_t b_pin, uint8_t buzz_pin) {
    this->r_pin = r_pin;
    this->g_pin = g_pin;
    this->b_pin = b_pin;
    this->buzz_pin = buzz_pin;

    pr = pb = pg = br = bg = bb = 0;
    pulse_duration = pulse_interval = 0;
    pulse_on = pulsed = false;
    pulse_ref = 0;
}

void Indicator::start() {
    pinMode(r_pin, OUTPUT);
    pinMode(g_pin, OUTPUT);
    pinMode(b_pin, OUTPUT);
    pinMode(buzz_pin, OUTPUT);

    analogWrite(r_pin, PWM_MAX);
    analogWrite(g_pin, PWM_MAX);
    analogWrite(b_pin, PWM_MAX);
}

void Indicator::setPassiveRGB(uint8_t r, uint8_t g, uint8_t b) {
    pr = ((uint16_t) r) << 4U;
    pg = ((uint16_t) g) << 4U;
    pb = ((uint16_t) b) << 4U;

    analogWrite(r_pin, constrain(PWM_MAX - r, 0, PWM_MAX));
    analogWrite(g_pin, constrain(PWM_MAX - g, 0, PWM_MAX));
    analogWrite(b_pin, constrain(PWM_MAX - b, 0, PWM_MAX));
}

void Indicator::setBlinkRGB(uint8_t r, uint8_t g, uint8_t b) {
    br = ((uint16_t) r) << 4U;
    bg = ((uint16_t) g) << 4U;
    bb = ((uint16_t) b) << 4U;
}

void Indicator::cycle() {
    for (int r = 0; r < PWM_MAX; r += 64)
        for (int g = 0; g < PWM_MAX; g += 64)
            for (int b = 0; b < PWM_MAX; b += 64) {
                setPassiveRGB(r, g, b);
                delay(1);
            }
}

void Indicator::update() {
    if (pulse_on) {
        unsigned long t = (millis() - pulse_ref) % (pulse_duration + pulse_interval);

        if (t < pulse_duration && !pulsed) {
            beep(pulse_duration);
            pulsed = true;

            analogWrite(r_pin, constrain(PWM_MAX - br, 0, PWM_MAX));
            analogWrite(g_pin, constrain(PWM_MAX - bg, 0, PWM_MAX));
            analogWrite(b_pin, constrain(PWM_MAX - bb, 0, PWM_MAX));
        } else if (t >= pulse_duration && pulsed) {
            pulsed = false;

            analogWrite(r_pin, constrain(PWM_MAX - pr, 0, PWM_MAX));
            analogWrite(g_pin, constrain(PWM_MAX - pg, 0, PWM_MAX));
            analogWrite(b_pin, constrain(PWM_MAX - pb, 0, PWM_MAX));
        }

    }
}

void Indicator::beep(unsigned int duration) const {
    tone(buzz_pin, F_BEEP, duration);
}

void Indicator::boop(unsigned int duration) const {
    tone(buzz_pin, F_BOOP, duration);
}

void Indicator::silence() const {
    noTone(buzz_pin);
}

void Indicator::setPulse(unsigned int duration, unsigned int interval) {
    this->pulse_duration = duration;
    this->pulse_interval = interval;
    this->pulse_on = true;
    this->pulsed = false;
    pulse_ref = millis();
}

void Indicator::disablePulse() {
    this->pulse_on = false;
}

void Indicator::beepstring(uint8_t bitstring, int bitrate) {
    unsigned int duration = 4 * 1000 / bitrate / 5;
    unsigned int wait = 1000 / bitrate;

    for (int i = 0; i < sizeof(uint8_t) * 8; i++) {
        if (bitstring & 0x80U)
            beep(duration);
        else
            boop(duration);
        delay(wait);

        bitstring <<= 1U;
    }
}

void Indicator::beepstring(uint16_t bitstring, int bitrate) {
    unsigned int duration = 4 * 1000 / bitrate / 5;
    unsigned int wait = 1000 / bitrate;

    for (int i = 0; i < sizeof(uint16_t) * 8; i++) {
        if (bitstring & 0x8000U)
            beep(duration);
        else
            boop(duration);
        delay(wait);

        bitstring <<= 1U;
    }
}

void Indicator::beepstring(uint32_t bitstring, int bitrate) {
    unsigned int duration = 4 * 1000 / bitrate / 5;
    unsigned int wait = 1000 / bitrate;

    for (int i = 0; i < sizeof(uint32_t) * 8; i++) {
        if (bitstring & 0x80000000UL)
            beep(duration);
        else
            boop(duration);
        delay(wait);

        bitstring <<= 1U;
    }
}

void Indicator::beepstring(uint64_t bitstring, int bitrate) {
    unsigned int duration = 4 * 1000 / bitrate / 5;
    unsigned int wait = 1000 / bitrate;

    for (int i = 0; i < sizeof(uint64_t) * 8; i++) {
        if (bitstring & 0x8000000000000000UL)
            beep(duration);
        else
            boop(duration);
        delay(wait);

        bitstring <<= 1U;
    }
}

