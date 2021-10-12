//
// Created by Misha on 5/5/2021.
//

#include "ZSS.h"

ZSS::ZSS(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t l1, uint8_t l2) {
    deploying = false;
    current_active = false;
    timeout_ref = 0;

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
    timeout_ref = millis();
}

void ZSS::retract() {
    digitalWrite(p1, HIGH);
    digitalWrite(p4, HIGH);
    digitalWrite(p2, LOW);
    digitalWrite(p3, LOW);

    deploying = false;
    current_active = true;
    timeout_ref = millis();
}

void ZSS::deploy() {
    digitalWrite(p2, HIGH);
    digitalWrite(p3, HIGH);
    digitalWrite(p1, LOW);
    digitalWrite(p4, LOW);

    deploying = true;
    current_active = true;
    timeout_ref = millis();
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

    current_active = false;
}

void ZSS::update() {
    if(current_active && millis() - timeout_ref >= ZSS_CURRENT_TIMEOUT)
        halt();
}
