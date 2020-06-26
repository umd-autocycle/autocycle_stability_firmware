//
// Created by Misha on 6/25/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_TONE_H
#define AUTOCYCLE_STABILITY_FIRMWARE_TONE_H


#include "Arduino.h"

void tone(uint32_t _pin, uint32_t frequency, uint32_t duration = 0);

void noTone(uint32_t _pin);


#endif //AUTOCYCLE_STABILITY_FIRMWARE_TONE_H
