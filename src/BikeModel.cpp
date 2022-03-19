//
// Created by Misha on 12/24/2020.
//

#include "BikeModel.h"

BikeModel::BikeModel() {
    /* Bicycle parameter definitions */
    float w = 1.16;
    float t = 0.09;
    float alpha = 1.319;
    float g = 9.81;

    // Rear wheel parameters
    float r_rw = 0.35;                           // Rear wheel radius
    float m_rw = 3.30;                           // Rear wheel mass
    BLA::Matrix<3, 1> A = {0.177, 0.354, 0.177};   // Rear wheel moment of inertia

    // Rear frame parameters
    float x_arf = 0.4;
    float y_arf = 0;
    float z_arf = -0.605;
    float m_arf = 28.65;                          // Rear frame mass
    BLA::Matrix<3, 3> B_arf = {                     // Rear frame moment of inertia
            3.124, 0, -0.877,
            0, 4.150, 0,
            -0.877, 0, 3.398,
    };

    // ZSS parameters
    float x_zss = 0;
    float y_zss = 0;
    float z_zss = -0.45; // GUESS
    float m_zss = 6.352;
    BLA::Matrix<3, 3> B_zss = {
            0.223, -0.045, -0.001,
            -0.045, 0.252, 0.002,
            -0.001, 0.002, 0.473
    };

    BLA::Matrix<3, 1> com_arf = {x_arf, y_arf, z_arf};
    BLA::Matrix<3, 1> com_zss = {x_zss, y_zss, z_zss};

    // Combine ZSS and Rear Frame MOI, COM, and Mass
    float m_rf = m_arf + m_zss;
    auto com_rf = (com_arf * m_arf + com_zss * m_zss) / m_rf;
    auto d_zss = com_zss - com_rf;
    auto d_arf = com_arf - com_rf;
    auto dd_zss = BLA::Identity<3, 3>() * BLA::Norm(d_zss) * BLA::Norm(d_zss) - d_zss * (~d_zss);
    auto dd_arf = BLA::Identity<3, 3>() * BLA::Norm(d_arf) * BLA::Norm(d_arf) - d_arf * (~d_arf);
    auto B = B_arf + B_zss + dd_arf * m_arf + dd_zss * m_zss;
    float x_rf = com_rf(0);
    float y_rf = com_rf(1);
    float z_rf = com_rf(2);

    // Front frame parameters
    float x_ff = 0.92;
    float y_ff = 0;
    float z_ff = -0.835;
    float m_ff = 3.05;                           // Front frame mass
    BLA::Matrix<3, 3> C = {                     // Front frame moment of inertia
            0.344, 0, 0.0637,
            0, 0.239, 0,
            0.0637, 0, 0.0578,
    };

    // Front wheel parameters
    float r_fw = 0.35;                          // Front wheel radius
    float m_fw = 2.9;                           // Front wheel mass
    BLA::Matrix<3, 1> D = {0.177, 0.354, 0.177};   // Front wheel moment of inertia


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

    // Newly ID'd system dynamics
    M = {
            26.67504339, 1.21856943,
            1.21856943, 0.59438128
    };
    C1 = {
            0., 4.97370516,
            -0.98015773, 2.43085255
    };
    K0 = {
            -210.6481775, 1.14387605,
            1.14387605, 3.2817143
    };
    K2 = {
            0., 21.88145723,
            0., -0.86196881
    };

    M_inv = BLA::Inverse(M);
}

BLA::Matrix<4, 4> BikeModel::dynamicsMatrix(float v, bool free_running) {
    if (free_running) {
        auto a11 = BLA::Zeros<2, 2>();
        auto a12 = BLA::Identity<2, 2>();
        auto a21 = M_inv * (K0 + K2 * (v * v)) * (-1);
        auto a22 = M_inv * C1 * (-v);
        return (a11 || a12) && (a21 || a22);
    } else {
        auto a11 = BLA::Zeros<2, 2>();
        auto a12 = BLA::Identity<2, 2>();
        auto a21 = BLA::Zeros<2, 2>();
        auto a22 = BLA::Zeros<2, 2>();
        return (a11 || a12) && (a21 || a22);
    }
}

BLA::Matrix<4, 2> BikeModel::controlsMatrix(float v, bool free_running) {
    if (free_running)
        return BLA::Zeros<2, 2>() && M_inv;
    else
        return BLA::Zeros<2, 2>() && BLA::Zeros<2, 2>();
}

BLA::Matrix<4, 4> BikeModel::kalmanTransitionMatrix(float v, float dt, bool free_running) {
    return (BLA::Identity<4, 4>() + dynamicsMatrix(v, free_running) * dt);
}

BLA::Matrix<4, 2> BikeModel::kalmanControlsMatrix(float v, float dt, bool free_running) {
    return controlsMatrix(v, free_running) * dt;
}
