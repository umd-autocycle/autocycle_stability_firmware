//
// Created by Misha on 10/1/2021.
//

#include "Encoder.h"

Encoder::Encoder(int aPin, int bPin) {
    this->aPin = aPin;
    this->bPin = bPin;
    curA = prevA = 0;
    curB = prevB = 0;
    combined = 0;
}

void Encoder::start() {
    pinMode(aPin, INPUT_PULLUP);
    pinMode(bPin, INPUT_PULLUP);

    curA = prevA = digitalRead(aPin);
    curB = prevB = digitalRead(bPin);
}

float Encoder::getSpeed() {
    int pulses = sum_counter();
    float pulse_per_s = ((float) pulses / ((float) AVG_PERIOD_MS / 1000.0f));
    float rps = pulse_per_s / ((float) PPR);
    float rear_per_sprocket = (float) SPROCKET_TEETH / (float) REAR_TEETH;

    return rps * rear_per_sprocket * WHEEL_CIRCUMFERENCE;
}

void Encoder::update() {
    int bin = (int) (millis() % AVG_PERIOD_MS) / (AVG_PERIOD_MS / ENC_BIN_COUNT);
    if (bin != current_bin) {
        current_bin = bin;
        counter[current_bin] = 0;
    }
}

int Encoder::sum_counter() {
    int acc = 0;
    for (int i : counter)
        acc += i;

    return acc;
}

void Encoder::countPulse() {
    int bin = (int) (millis() % AVG_PERIOD_MS) / (AVG_PERIOD_MS / ENC_BIN_COUNT);
    if (bin != current_bin) {
        current_bin = bin;
        counter[current_bin] = 0;
    }

    curA = digitalRead(aPin);
    curB = digitalRead(bPin);
    combined = (curA << 3) | (curB << 2) | (prevA << 1) | prevB;

    switch (combined) {
        case 0b1100:    // Clockwise
        case 0b0011:
        case 0b1000:
        case 0b0111:
            counter[current_bin]++;
            break;

        case 0b1001:    // Counter-clockwise
        case 0b0110:
        case 0b0010:
        case 0b1101:
            counter[current_bin]--;
            break;

        default:        // All other cases invalid
            break;
    }

    prevA = curA;
    prevB = curB;
}
