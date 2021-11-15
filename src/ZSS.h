//
// Created by Misha on 5/5/2021.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_ZSS_H
#define AUTOCYCLE_STABILITY_FIRMWARE_ZSS_H

#include <Arduino.h>
#include <Servo.h>

#define ZSS_RETRACT_US      2000

class ZSS {
public:
    ZSS(uint8_t p1, uint8_t p2, int a1_offset, int a2_offset);

    void start();
    void deploy();
    void retract();
    void halt();

private:
    uint8_t p1, p2;
    int a1_offset, a2_offset;
    Servo *serv1, *serv2;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_ZSS_H
