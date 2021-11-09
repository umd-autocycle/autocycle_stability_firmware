//
// Created by Misha on 12/5/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H
#define AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H

#include <BasicLinearAlgebra.h>
#include "ExtraLinalg.h"

//using namespace BLA;

template<int xN, int yN, int uN>
class KalmanFilter {
public:
    void predict(BLA::Matrix<uN, 1> u);

    void update(BLA::Matrix<yN, 1> y);


    BLA::Matrix<xN, 1, BLA::Array<xN, 1>> x;   // State estimate
    BLA::Matrix<xN, xN, BLA::Array<xN, xN>> P_2;  // State estimate covariance matrix sqrt

    BLA::Matrix<xN, xN, BLA::Array<xN, xN>> A;  // State transition matrix
    BLA::Matrix<xN, uN, BLA::Array<xN, uN>> B;  // Control matrix
    BLA::Matrix<yN, xN, BLA::Array<yN, xN>> C;  // Sensor matrix

    BLA::Matrix<xN, xN, BLA::Array<xN, xN>> Q_2;  // Process covariance matrix sqrt
    BLA::Matrix<yN, yN, BLA::Array<yN, yN>> R_2;  // Measurement covariance matrix sqrt
};


template<int xN, int yN, int uN>
void KalmanFilter<xN, yN, uN>::predict(BLA::Matrix<uN, 1> u) {
    x = A * x + B * u;
    BLA::Matrix<2 * xN, xN> newP2;
    BLA::Matrix<2 * xN, 2 * xN> throwaway;
    BLA::Matrix<2 * xN, xN> takeop = ~((A * P_2) || Q_2);

    qr_decomposition<2 * xN, xN>(takeop, throwaway, newP2);
    P_2 = ~(newP2.template Submatrix<xN, xN>(0, 0));
}

template<int xN, int yN, int uN>
void KalmanFilter<xN, yN, uN>::update(BLA::Matrix<yN, 1> y) {
    auto residual = y - C * x;

    BLA::Matrix<xN + yN, xN + yN> M =
            (BLA::Matrix<yN, xN + yN>) ((~R_2) || BLA::Zeros<yN, xN>()) && (~(C * P_2) || ~P_2);

    BLA::Matrix<xN + yN, xN + yN> throwaway, S;
    qr_decomposition<xN + yN, xN + yN>(M, throwaway, S);
    BLA::Matrix<xN, yN> K = ~(S.template Submatrix<yN, xN>(0, yN));
    BLA::Matrix<yN, yN> N = ~(S.template Submatrix<yN, yN>(0, 0));

    x = x + K * (BLA::Inverse(N)) * residual;
    P_2 = ~(S.template Submatrix<xN, xN>(yN, yN));
}


#endif //AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H
