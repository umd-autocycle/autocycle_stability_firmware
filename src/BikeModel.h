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

    BLA::Matrix<2, 2> M{};    // Equivalent mass matrix
    BLA::Matrix<2, 2> M_inv{};
    BLA::Matrix<2, 2> C1{};   // Linear-velocity equivalent damping matrix
    BLA::Matrix<2, 2> K0{};   // Constant equivalent stiffness matrix
    BLA::Matrix<2, 2> K2{};   // Velocity-squared equivalent stiffness matrix
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_BIKEMODEL_H
