//
// Created by Misha on 10/1/2021.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_ENCODER_H
#define AUTOCYCLE_STABILITY_FIRMWARE_ENCODER_H

#include <Arduino.h>
#include "DriveMotor.h"

#define ENC_BIN_COUNT   125
#define AVG_PERIOD_MS   250
#define SPROCKET_TEETH  14
#define PPR             360

class Encoder {
public:
    Encoder(int aPin, int bPin);

    float getSpeed();

    void start();

    void update();

    void countPulse();

private:
    int sum_counter();

    volatile int16_t counter[ENC_BIN_COUNT]{};
    volatile int current_bin = 0;
    int aPin, bPin;
    volatile int prevA, prevB;
    volatile int curA, curB;
    volatile byte combined;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_ENCODER_H
