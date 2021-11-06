//
// Created by Misha on 5/5/2021.
//

#include "ZSS.h"

ZSS::ZSS(uint8_t p1, uint8_t p2, int a1_offset, int a2_offset) {
    this->p1 = p1;
    this->p2 = p2;
    this->a1_offset = a1_offset;
    this->a2_offset = a2_offset;

    serv1 = new Servo();
    serv2 = new Servo();
}

void ZSS::start() {
    pinMode(p1, OUTPUT);
    pinMode(p2, OUTPUT);
    serv1->attach(p1);
    serv2->attach(p2);
}

void ZSS::retract() {
    serv1->writeMicroseconds(ZSS_RETRACT_US);
    serv2->writeMicroseconds(ZSS_RETRACT_US);
}

void ZSS::deploy() {
    serv1->writeMicroseconds(a1_offset);
    serv2->writeMicroseconds(a2_offset);
}

void ZSS::halt() {
    serv1->writeMicroseconds(0);
    serv2->writeMicroseconds(0);
}
