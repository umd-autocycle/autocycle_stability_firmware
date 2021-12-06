//
// Created by Misha on 3/9/2021.
//

#include "FSFController.h"
#include <BasicLinearAlgebra.h>


FSFController::FSFController(BikeModel *model, float torque_max, float l1, float l2, float l3, float l4) {
    this->model = model;
    this->torque_max = torque_max;

    this->l1 = l1;
    this->l2 = l2;
    this->l3 = l3;
    this->l4 = l4;

    M_det = BLA::Determinant(model->M);
    C1_det = BLA::Determinant(model->C1);
    K0_det = BLA::Determinant(model->K0);
    K2_det = BLA::Determinant(model->K2);
    BLA::Matrix<2, 2> t1 = model->C1 - model->M;
    C1_M_det = BLA::Determinant(t1);
    BLA::Matrix<2, 2> t2 = model->K2 - model->M;
    K2_M_det = BLA::Determinant(t2);
    BLA::Matrix<2, 2> t3 = model->K0 - model->M;
    K0_M_det = BLA::Determinant(t3);
    BLA::Matrix<2, 2> t4 = model->C1 - model->K2;
    C1_K2_det = BLA::Determinant(t4);
    BLA::Matrix<2, 2> t5 = model->C1 - model->K0;
    C1_K0_det = BLA::Determinant(t5);
    BLA::Matrix<2, 2> t6 = model->K0 - model->K2;
    K0_K2_det = BLA::Determinant(t6);
}

float
FSFController::control(float phi, float del, float dphi, float ddel, float phi_r, float del_r, float v, float dt) {
    BLA::Matrix<4, 1> x = {phi, del, dphi, ddel};
    BLA::Matrix<4, 4> LHS = {
            Qk1(l1, v), Qk2(l1, v), Qk3(l1, v), Qk4(l1, v),
            Qk1(l2, v), Qk2(l2, v), Qk3(l2, v), Qk4(l2, v),
            Qk1(l3, v), Qk2(l3, v), Qk3(l3, v), Qk4(l3, v),
            Qk1(l4, v), Qk2(l4, v), Qk3(l4, v), Qk4(l4, v),
    };
    BLA::Matrix<4, 1> RHS = {
            -RHSe(l1, v),
            -RHSe(l2, v),
            -RHSe(l3, v),
            -RHSe(l4, v)
    };

    BLA::Matrix<4, 1> K = BLA::Inverse(LHS) * RHS;
    K(3,0) = 0;

    float u = -((~K) * x)(0, 0);

    return constrain(u, -torque_max, torque_max);
}

float FSFController::Qk1(float l, float v) {
    return -(model->M(0, 1) * l * l + model->C1(0, 1) * l * v + model->K2(0, 1) * v * v + model->K0(0, 1));
}

float FSFController::Qk2(float l, float v) {
    return model->M(0, 0) * l * l + model->C1(0, 0) * l * v + model->K2(0, 0) * v * v + model->K0(0, 0);
}

float FSFController::Qk3(float l, float v) {
    return -(model->M(0, 1) * l * l + model->C1(0, 1) * l * v + model->K2(0, 1) * v * v + model->K0(0, 1)) * l;
}

float FSFController::Qk4(float l, float v) {
    return (model->M(0, 0) * l * l + model->C1(0, 0) * l * v + model->K2(0, 0) * v * v + model->K0(0, 0)) * l;
}

float FSFController::RHSe(float l, float v) {
    return l * l * l * l * M_det + l * l * l * v * (C1_det + M_det - C1_M_det) +
           l * l * v * v * (C1_det + M_det + K2_det - K2_M_det)
           + l * l * (K0_det + M_det - K0_M_det) + l * v * v * v * (C1_det + K2_det - C1_K2_det)
           + l * v * (C1_det + K0_det - C1_K0_det) + v * v * v * v * K2_det
           + v * v * (K0_det + K2_det - K0_K2_det) + K0_det;
}

