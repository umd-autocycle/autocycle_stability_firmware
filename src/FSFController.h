//
// Created by Misha on 3/9/2021.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_FSFCONTROLLER_H
#define AUTOCYCLE_STABILITY_FIRMWARE_FSFCONTROLLER_H

#include "Controller.h"
#include "BikeModel.h"

class FSFController : public Controller {
public:
    FSFController(BikeModel *model, float torque_max, float l1, float l2, float l3, float l4);

    float control(float phi, float del, float dphi, float ddel, float phi_r, float del_r, float v, float dt) override;

private:
    BikeModel* model;
    float torque_max;
    float l1, l2, l3, l4;
    float M_det, C1_det, K0_det, K2_det, C1_M_det, K2_M_det, K0_M_det, C1_K2_det, C1_K0_det, K0_K2_det;
    float Qk1(float l, float v);
    float Qk2(float l, float v);
    float Qk3(float l, float v);
    float Qk4(float l, float v);
    float RHSe(float l, float v);
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_FSFCONTROLLER_H
