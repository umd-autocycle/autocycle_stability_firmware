//
// Created by Misha on 12/24/2020.
//

#include "BikeModel.h"

BikeModel::BikeModel() {
    /* Bicycle parameter definitions */
    float w = 1.02;
    float t = 0.08;
    float alpha = (float) atan(3.0);
    float g = 9.81;

    // Rear wheel parameters
    float r_rw = 0.3;                           // Rear wheel radius
    float m_rw = 2.0;                           // Rear wheel mass
    BLA::Matrix<3, 1> A = {0.06, 0.12, 0.06};   // Rear wheel moment of inertia

    // Rear frame parameters
    float x_rf = 0.3;
    float y_rf = 0;
    float z_rf = -0.9;
    float m_rf = 85.0;                          // Rear frame mass
    BLA::Matrix<3, 3> B = {                     // Rear frame moment of inertia
            9.2, 0, 2.4,
            0, 11, 0,
            2.4, 0, 2.8,
    };

    // Front frame parameters
    float x_ff = 0.9;
    float y_ff = 0;
    float z_ff = -0.7;
    float m_ff = 4.0;                           // Front frame mass
    BLA::Matrix<3, 3> C = {                     // Front frame moment of inertia
            0.0546, 0, -0.0162,
            0, 0.06, 0,
            -0.0162, 0, 0.0114,
    };

    // Front wheel parameters
    float r_fw = 0.35;                          // Front wheel radius
    float m_fw = 3.0;                           // Front wheel mass
    BLA::Matrix<3, 1> D = {0.14, 0.28, 0.14};   // Front wheel moment of inertia


    /* Computation of parameters for equivalent linearized matrices */
    float m_t = m_rw + m_rf + m_ff + m_fw;
    float x_t = (x_rf * m_rf + x_ff * m_ff + w * m_fw) / m_t;
    float z_t = (-r_rw * m_rw + z_rf * m_rf
                 + z_ff * m_ff - r_fw * m_fw) / m_t;

    float t_xx = A(0) + B(0, 0) + C(0, 0) + D(0)
                 + m_rw * (r_rw * r_rw) + m_rf * (z_rf * z_rf)
                 + m_fw * (r_fw * r_fw) + m_ff * (z_ff * z_ff);
    float t_xz = B(0, 2) + C(0, 2) - m_rf * x_rf * z_rf
                 - m_ff * x_ff * z_ff + m_fw * w * r_fw;
    float t_zz = A(2) + B(2, 2) + C(2, 2) + D(2) + m_rf * (x_rf * x_rf)
                 + m_ff * (x_ff * x_ff) + m_fw * (w * w);

    float m_f = m_ff + m_fw;
    float x_f = (x_ff * m_ff + w * m_fw) / m_f;
    float z_f = (z_ff * m_ff - r_fw * m_fw) / m_f;

    float f_xx = C(0, 0) + D(0) + m_ff * ((z_ff - z_f) * (z_ff - z_f))
                 + m_fw * ((r_fw + z_f) * (r_fw + z_f));
    float f_xz = C(0, 2) - m_ff * (x_ff - x_f) * (z_ff - z_f)
                 + m_fw * (w - x_f) * (r_fw + z_f);
    float f_zz = C(2, 2) + D(2) + m_ff * ((x_ff - x_f) * (x_ff - x_f))
                 + m_fw * ((w - x_f) * (w - x_f));

    float lam = PI / 2 - alpha;
    float u = (x_f - w - t) * cos(lam) - z_f * sin(lam);

    float f_ll = m_f * (u * u) + f_xx * (sin(lam) * sin(lam)) + 2 * f_xz * sin(lam) * cos(lam)
                 + f_zz * (cos(lam) * cos(lam));
    float f_lx = -m_f * u * z_f + f_xx * sin(lam) + f_xz * cos(lam);
    float f_lz = m_f * u * x_f + f_xz * sin(lam) + f_zz * cos(lam);

    float f = t * cos(lam) / w;

    float s_r = A(1) / r_rw;
    float s_f = D(1) / r_fw;
    float s_t = s_r + s_f;

    float s_u = m_f * u + f * m_t * x_t;

    /* Computation of equivalent linearized matrices */
    M = {
            t_xx, f_lx + f * t_xz,
            f_lx + f * t_xz, f_ll + 2 * f * f_lz + f * f * t_zz,
    };

    K0 = {
            g * m_t * z_t, -g * s_u,
            -g * s_u, -g * s_u * sin(lam),
    };

    K2 = {
            0, (s_t - m_t * z_t) * cos(lam) / w,
            0, (s_u + s_f * sin(lam)) * cos(lam) / w,
    };

    C1 = {
            0, f * s_t + s_f * cos(lam) + t_xz * cos(lam) / w - f * m_t * z_t,
            -(f * s_t + s_f * cos(lam)), f_lz * cos(lam) / w + f * (s_u + t_zz * cos(lam) / w),
    };

    M_inv = M.Inverse();

}

BLA::Matrix<4, 4> BikeModel::dynamicsMatrix(float v) {
    auto a11 = BLA::Zeros<2, 2>();
    auto a12 = BLA::Identity<2, 2>();
    auto a21 = M_inv * (K0 + K2 * (v * v)) * (-1);
    auto a22 = M_inv * C1 * (-v);
    return (a11 || a12) && (a21 || a22);
}

BLA::Matrix<4, 2> BikeModel::controlsMatrix(float v) {
    return BLA::Zeros<2, 2>() && M_inv;
}

BLA::Matrix<4, 4> BikeModel::kalmanTransitionMatrix(float v, float dt) {
    return (BLA::Identity<4, 4>() + dynamicsMatrix(v) * dt);
}

BLA::Matrix<4, 2> BikeModel::kalmanControlsMatrix(float v, float dt) {
    return controlsMatrix(v) * dt;
}
