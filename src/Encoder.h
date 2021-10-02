//
// Created by Misha on 10/1/2021.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_ENCODER_H
#define AUTOCYCLE_STABILITY_FIRMWARE_ENCODER_H

#include <Arduino.h>
#include "DriveMotor.h"

#define ENC_BIN_COUNT   10
#define AVG_PERIOD_MS   100
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
    volatile int16_t counter[ENC_BIN_COUNT]{};
    int current_bin = 0;
    int aPin, bPin;
    volatile int prevA, prevB;
    volatile int curA, curB;
    volatile byte combined;

    int sum_counter();
    int c = 0;

};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_ENCODER_H
