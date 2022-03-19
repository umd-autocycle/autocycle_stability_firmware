//
// Created by Misha on 12/24/2020.
// Bike model for linearized second order motion in phi and delta with constant velocity
// Derived from the work of Schwarm and Meijaard
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_BIKEMODEL_H
#define AUTOCYCLE_STABILITY_FIRMWARE_BIKEMODEL_H

#include <BasicLinearAlgebra.h>

class BikeModel {
public:
    BikeModel();

    BLA::Matrix<4, 4> dynamicsMatrix(float v, bool free_running);

    BLA::Matrix<4, 2> controlsMatrix(float v, bool free_running);

    BLA::Matrix<4, 4> kalmanTransitionMatrix(float v, float dt, bool free_running);

    BLA::Matrix<4, 2> kalmanControlsMatrix(float v, float dt, bool free_running);

    const BLA::Matrix<2, 2> M{
            26.67504339, 1.21856943,
            1.21856943, 0.59438128
    };    // Equivalent mass matrix
    BLA::Matrix<2, 2> M_inv;
    const BLA::Matrix<2, 2> C1{
            0., 4.97370516,
            -0.98015773, 2.43085255
    };   // Linear-velocity equivalent damping matrix
    const BLA::Matrix<2, 2> K0{
            -210.6481775, 1.14387605,
            1.14387605, 3.2817143
    };   // Constant equivalent stiffness matrix
    const BLA::Matrix<2, 2> K2{
            0., 21.88145723,
            0., -0.86196881
    };   // Velocity-squared equivalent stiffness matrix
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_BIKEMODEL_H
