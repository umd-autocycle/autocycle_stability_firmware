//
// Created by Misha on 7/1/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_TORQUEMOTOR_H
#define AUTOCYCLE_STABILITY_FIRMWARE_TORQUEMOTOR_H

#include "CANOpen.h"
#include <due_can.h>


// Motor data
#define POLE_PAIR_COUNT         6
#define MAX_PERM_CURRENT_MA     16000
#define RATED_CURRENT_MA        5330
#define MAX_CURRENT_DUR_MS      100
#define BLDC_MOTOR              0x0000000041U
#define RATED_TORQUE_N          0.5
#define GEARING                 33.0
#define EFFICIENCY              0.93

// Operating modes
#define OP_AUTO_SETUP           0xFEU
#define OP_PROFILE_TORQUE       0x04U


class TorqueMotor {
public:
    TorqueMotor(CANRaw *can_line, uint16_t node_id, unsigned int current_max, unsigned int torque_max,
                unsigned int torque_slope); // current and torque in thousandths of rated current and torque

    void start();

    void commission(); // Motor must be unloaded, not touched, and free to rotate in any direction.

    void torqueMode();

    void setTorque(double torque);

    void update();

private:
    CANOpenDevice *motor_dev;
    unsigned int torque_max, current_max, torque_slope;
    double desired_torque, actual_torque;
    BytesUnion outgoing;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_TORQUEMOTOR_H
