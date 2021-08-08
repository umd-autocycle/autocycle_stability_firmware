//
// Created by Misha on 5/5/2021.
//

#include "ZSS.h"

ZSS::ZSS(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t l1, uint8_t l2) {
    deploying = false;
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = p3;
    this->p4 = p4;
    this->l1 = l1;
    this->l2 = l2;
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

void ZSS::retract() {
    digitalWrite(p2, HIGH);
    digitalWrite(p3, HIGH);
    digitalWrite(p1, LOW);
    digitalWrite(p4, LOW);
    deploying = false;
}

void ZSS::deploy() {
    digitalWrite(p1, HIGH);
    digitalWrite(p4, HIGH);
    digitalWrite(p2, LOW);
    digitalWrite(p3, LOW);
    deploying = true;
    if (!digitalRead(l1) || !digitalRead(l2))
        halt();
}

bool ZSS::retracted() {
    return !deploying;
}

void ZSS::halt() {
    digitalWrite(p1, HIGH);
    digitalWrite(p2, HIGH);
    digitalWrite(p3, HIGH);
    digitalWrite(p4, HIGH);
}
