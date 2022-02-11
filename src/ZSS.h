//
// Created by Misha on 5/5/2021.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_ZSS_H
#define AUTOCYCLE_STABILITY_FIRMWARE_ZSS_H

#include <Arduino.h>
#include <Servo.h>

#define ZSS_RETRACT_US      2000

#define ZSS_DEPLOY_MS       4500

class ZSS {
public:
    ZSS(uint8_t p1, uint8_t p2);

    void start(unsigned int a1_offset, unsigned int a2_offset);
    void deploy();
    void retract();
    void halt();
    void run();

    void adjustOffsets(unsigned int a1_off, unsigned int a2_off);

    bool deploying;
    unsigned int a1_offset{}, a2_offset{};

private:
    uint8_t p1, p2;
    unsigned int a1_start{}, a2_start{};
    unsigned int a1_target{}, a2_target{};
    unsigned int a1_setting{}, a2_setting{};
    unsigned long move_commence;
    Servo *serv1, *serv2;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_ZSS_H
