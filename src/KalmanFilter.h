//
// Created by Misha on 12/5/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H
#define AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H

#include <BasicLinearAlgebra.h>

using namespace BLA;

template<int xN, int yN, int uN>
class KalmanFilter {
public:
    void predict(BLA::Matrix<uN, 1, Array<uN, 1>> u);

    void update(BLA::Matrix<yN, 1, Array<yN, 1>> y);


    BLA::Matrix<xN, 1, Array<xN, 1>> x;   // State estimate
    BLA::Matrix<xN, xN, Array<xN, xN>> P;  // State estimate covariance matrix

    BLA::Matrix<xN, xN, Array<xN, xN>> A;  // State transition matrix
    BLA::Matrix<xN, uN, Array<xN, uN>> B;  // Control matrix
    BLA::Matrix<yN, xN, Array<yN, xN>> C;  // Sensor matrix

    BLA::Matrix<xN, xN, Array<xN, xN>> Q;  // Process covariance matrix
    BLA::Matrix<yN, yN, Array<yN, yN>> R;  // Measurement covariance matrix
};


template<int xN, int yN, int uN>
void KalmanFilter<xN, yN, uN>::predict(BLA::Matrix<uN, 1, Array<uN, 1>> u) {
    x = A * x + B * u;
    P = A * P * (~A) + Q;
}

template<int xN, int yN, int uN>
void KalmanFilter<xN, yN, uN>::update(BLA::Matrix<yN, 1, Array<yN, 1>> y) {
    auto residual = y - C * x;
    auto s = C * P * (~C) + R;
    auto K = P * (~C) * (BLA::Inverse(s));
    x = x + K * residual;
    P = (BLA::Identity<xN, xN>() - K * C) * P;
}


#endif //AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H
