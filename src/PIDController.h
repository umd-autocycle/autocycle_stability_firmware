//
// Created by Misha on 12/10/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_PIDCONTROLLER_H
#define AUTOCYCLE_STABILITY_FIRMWARE_PIDCONTROLLER_H

#include "Controller.h"

class PIDController : public Controller {
public:
    PIDController(float k_p, float k_i, float k_d, float torque_max);

    float control(float phi, float del, float dphi, float ddel, float phi_r, float del_r, float dt) override;

private:
    float ei = 0;
    float k_p, k_i, k_d;
    float torque_max;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_PIDCONTROLLER_H
