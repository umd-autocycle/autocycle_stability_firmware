//
// Created by Misha on 12/10/2020.
//

#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float k_p, float k_i, float k_d, float torque_max) {
    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
    this->torque_max = torque_max;
}

float PIDController::control(float phi, float del, float dphi, float ddel, float phi_r, float del_r, float dt) {
    float e = phi_r - phi;
    float de = -dphi;
    ei += e * dt;

    return min(k_p * e + k_i * ei + k_d * de, torque_max);
}
