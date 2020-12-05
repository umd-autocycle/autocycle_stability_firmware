//
// Created by Misha on 12/5/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H
#define AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H

#include <BasicLinearAlgebra.h>

class KalmanFilter {
public:
    BLA::Matrix<6, 1>
    filter(double measuredphi, double doublemeasureddelta, double doublemeasureddphi, double doublemeasuredddelta,
           double doublemeasuredvelocity, double doubleappliedTorque, double doublemeasuredax, double doublemeasureday);

private:
    BLA::Matrix<6, 1> x_old;
    BLA::Matrix<6, 6> P_old;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_KALMANFILTER_H
