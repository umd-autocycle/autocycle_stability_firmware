//
// Created by Misha on 12/5/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H
#define AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class KalmanFilter {
public:
    KalmanFilter(BLA::Matrix<6, 6> A, BLA::Matrix<6, 3>);
    BLA::Matrix<6, 1>
    filter(float measured_phi, float measured_delta, float measured_dphi, float measured_ddelta,
           float measured_velocity, float applied_torque, float measured_ax, float measured_ay);

private:
    BLA::Matrix<6, 1> x_old;
    BLA::Matrix<6, 6> P_old;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H
