//
// Created by Misha on 5/5/2021.
//

#include "ZSS.h"

ZSS::ZSS(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
    down = false;
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = p3;
    this->p4 = p4;


}

void ZSS::start() {
    pinMode(p1, OUTPUT);
    pinMode(p2, OUTPUT);
    pinMode(p3, OUTPUT);
    pinMode(p4, OUTPUT);

    digitalWrite(p1, HIGH);
    digitalWrite(p2, HIGH);
    digitalWrite(p3, HIGH);
    digitalWrite(p4, HIGH);
}

void ZSS::deploy() {
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
    digitalWrite(p3, HIGH);
    digitalWrite(p4, LOW);
    down = true;
}

void ZSS::retract() {
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
    digitalWrite(p3, LOW);
    digitalWrite(p4, HIGH);
    down = false;
}

bool ZSS::retracted() {
    return down;
}
