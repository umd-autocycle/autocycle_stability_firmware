//
// Created by Misha on 11/8/2021.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_EXTRALINALG_H
#define AUTOCYCLE_STABILITY_FIRMWARE_EXTRALINALG_H

#include <BasicLinearAlgebra.h>

// Householder QR decomposition
template<int m, int n>
bool qr_decomposition(BLA::Matrix<m, n> A, BLA::Matrix<m, m> &Q, BLA::Matrix<m, n> &R) {
    if (m < n) {
        return false;
    }

    Q = BLA::Identity<m, m>();
    R = A;

    for (int j = 0; j < n; j++) {
//        auto subm1 = R.template Submatrix<m - j, 1>(j, j);
//        auto subm2 = R.template Submatrix<m - j, n>(j, 0);

        float acc = 0;
        for (int i = j; i < m; i++)
            acc += R(i, j) * R(i, j);
        auto normx = sqrt(acc);

        float s = R(j, j) < 0 ? 1 : -1;
        float u1 = R(j, j) - s * normx;

//        auto w = subm1 / u1;
//        w(1) = 1;
        float tau = -s * u1 / normx;

        //  subm2 -= (tau * w) * ((~w) * subm2);
        for (int i = j; i < m; i++) {
            for (int k = 0; k < n; k++) {
                acc = 0;
                for (int l = 0; l < n; l++) {
                    float to_add = R(i, j) * R(k, j) * R(k, l);
                    if(i == 1)
                        to_add /= R(i, j);
                    if(k == 1)
                        to_add /= R(k, j);

                    acc += to_add;
                }

                R(i, k) -= tau * acc;
            }
        }
        // auto subm3 = Q.template Submatrix<m, m - j>(0, j);
        // subm3 -= (subm3 * w) * ~(tau * w);
//        for (int i = 0; i < m; i++) {
//            for (int k = j; j < m; j++) {
//                Q
//            }
//        }
    }

    return true;
};

#endif //AUTOCYCLE_STABILITY_FIRMWARE_EXTRALINALG_H
