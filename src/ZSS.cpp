//
// Created by Misha on 5/5/2021.
//

#include "ZSS.h"

ZSS::ZSS(uint8_t p1, uint8_t p2, int a1_offset, int a2_offset) {
    this->p1 = p1;
    this->p2 = p2;
    this->a1_offset = a1_offset;
    this->a2_offset = a2_offset;
    deploying = true;
    move_commence = 0;
    a1_setting = a1_target = a1_start = a1_offset;
    a2_setting = a2_target = a2_start = a2_offset;

    serv1 = new Servo();
    serv2 = new Servo();
}

void ZSS::start() {
    pinMode(p1, OUTPUT);
    pinMode(p2, OUTPUT);
    serv1->attach(p1);
    serv2->attach(p2);

    serv1->writeMicroseconds(a1_setting);
    serv2->writeMicroseconds(a2_setting);
}

void ZSS::retract() {
    if(deploying) {
        move_commence = millis();

        deploying = false;
        a1_start = a1_offset;
        a2_start = a2_offset;
        a1_target = ZSS_RETRACT_US;
        a2_target = ZSS_RETRACT_US;
    }
}

void ZSS::deploy() {
    if (!deploying){
        move_commence = millis();

        deploying = true;
        a1_start = ZSS_RETRACT_US;
        a2_start = ZSS_RETRACT_US;
        a1_target = a1_offset;
        a2_target = a2_offset;
    }
}

void ZSS::halt() {
    serv1->writeMicroseconds(0);
    serv2->writeMicroseconds(0);
}

void ZSS::run() {
    int move_time = (int) (millis() - move_commence);
    if (move_time < ZSS_DEPLOY_MS) {
        a1_setting = map(move_time, 0, ZSS_DEPLOY_MS, a1_start, a1_target);
        a2_setting = map(move_time, 0, ZSS_DEPLOY_MS, a2_start, a2_target);

        serv1->writeMicroseconds(a1_setting);
        serv2->writeMicroseconds(a2_setting);
    }
}
