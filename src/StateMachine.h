//
// Created by Misha on 12/23/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_STATEMACHINE_H
#define AUTOCYCLE_STABILITY_FIRMWARE_STATEMACHINE_H


// State actions
void idle();

void calibrate();

void manual();

void assist();

void automatic();

void fallen();

void emergency_stop();


#endif //AUTOCYCLE_STABILITY_FIRMWARE_STATEMACHINE_H
