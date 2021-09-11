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
    void predict(BLA::Matrix<uN, 1, Array<uN, 1, double>> u);

    void update(BLA::Matrix<yN, 1, Array<yN, 1, double>> y);


    BLA::Matrix<xN, 1, Array<xN, 1, double>> x;   // State estimate
    BLA::Matrix<xN, xN, Array<xN, xN, double>> P;  // State estimate covariance matrix

    BLA::Matrix<xN, xN, Array<xN, xN, double>> A;  // State transition matrix
    BLA::Matrix<xN, uN, Array<xN, uN, double>> B;  // Control matrix
    BLA::Matrix<yN, xN, Array<yN, xN, double>> C;  // Sensor matrix

    BLA::Matrix<xN, xN, Array<xN, xN, double>> Q;  // Process covariance matrix
    BLA::Matrix<yN, yN, Array<yN, yN, double>> R;  // Measurement covariance matrix
};


template<int xN, int yN, int uN>
void KalmanFilter<xN, yN, uN>::predict(BLA::Matrix<uN, 1, Array<uN, 1, double>> u) {
    x = A * x + B * u;
    P = A * P * (~A) + Q;
}

template<int xN, int yN, int uN>
void KalmanFilter<xN, yN, uN>::update(BLA::Matrix<yN, 1, Array<yN, 1, double>> y) {
    auto residual = y - C * x;
    auto S = C * P * (~C) + R;
    auto K = P * (~C) * (S.Inverse());
    x = x + K * residual;
    P = (BLA::Identity<xN, xN>() - K * C) * P;
}


#endif //AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H
