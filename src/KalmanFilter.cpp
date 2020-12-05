//
// Created by Misha on 12/5/2020.
//

#include "KalmanFilter.h"
#include <BasicLinearAlgebra.h>

#define TIMERATE .1

using namespace BLA;

BLA::Matrix<6, 6> buildA(double);

BLA::Matrix<6, 6> buildA(double v) {
    BLA::Matrix<6, 6> A;
    A << 0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            9.4702, -0.5888 - 0.8868 * v * v, -0.104 * v, -0.3277 * v, 0, 0,
            12.3999, 31.5587 - 2.0423 * v * v, 3.6177 * v, -3.1388 * v, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;
    return A;
}

BLA::Matrix<6, 1> KalmanFilter::filter(double measuredphi, double measureddelta, double measureddphi,
                                       double measuredddelta, double measuredvelocity,
                                       double appliedTorque, double measuredax, double measureday) {
    BLA::Matrix<6, 6> A;
    BLA::Matrix<6, 3> B;
    BLA::Matrix<6, 6> C;
    BLA::Matrix<6, 3> D;
    BLA::Matrix<6, 1> x_new;

    BLA::Matrix<6, 1> x_prost;
    BLA::Matrix<6, 1> y_new;
    BLA::Matrix<3, 1> u_new;
    BLA::Matrix<6, 6> P_new;
    BLA::Matrix<6, 6> P_prost;
    BLA::Matrix<6, 6> K;
    BLA::Matrix<6, 6> Q;
    BLA::Matrix<6, 6> R;
    BLA::Matrix<6, 6> I;
    B << 0, 0, 0,
            0, 0, 0,
            -0.1226, 0, 0,
            4.265, 0, 0,
            0, 1, 0,
            0, 0, 1;
    C << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
    D.Fill(0);
    I << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
    x_old << .2, 0, 0, 0, 0, 0;
    P_old << 100, 0, 0, 0, 0, 0,
            0, 100, 0, 0, 0, 0,
            0, 0, 100, 0, 0, 0,
            0, 0, 0, 100, 0, 0,
            0, 0, 0, 0, 100, 0,
            0, 0, 0, 0, 0, 100;
    Q << 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
    R << -.1, 0, 0, 0, 0, 0,
            0, -.1, 0, 0, 0, 0,
            0, 0, -.1, 0, 0, 0,
            0, 0, 0, -.1, 0, 0,
            0, 0, 0, 0, .1, 0,
            0, 0, 0, 0, 0, .1;

    //set the values
    y_new << measuredphi, measureddelta, measureddphi, measuredddelta, cos(measureddelta) * measuredvelocity,
            sin(measureddelta) * measuredvelocity;
    u_new << appliedTorque, measuredax, measureday;
//    A = buildA(v[i]) * TIMERATE + I;

    //cout << "set prediction" << endl
    x_prost = A * x_old + B * u_new;
    P_prost = A * P_old * ~A + Q;
    K = P_prost * C * (C * P_prost * ~C + R).Inverse();
    x_new = x_prost + K * (y_new - C * x_prost);
    P_new = (I - K * C) * P_prost;

    //new to old
    x_old = x_new;
    P_old = P_new;
    return x_new;
}
