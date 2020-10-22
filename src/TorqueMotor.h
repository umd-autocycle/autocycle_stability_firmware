//
// Created by Misha on 7/1/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_TORQUEMOTOR_H
#define AUTOCYCLE_STABILITY_FIRMWARE_TORQUEMOTOR_H

#include "CANOpen.h"
#include <due_can.h>

// Operating modes
#define OP_NONE                 0x00U
#define OP_AUTO_SETUP           0xFEU
#define OP_PROFILE_TORQUE       0x04U
#define OP_PROFILE_VELOCITY     0x03U
#define OP_PROFILE_POSITION     0x01U

class TorqueMotor {
public:
    TorqueMotor(CANRaw *can_line, uint16_t node_id, unsigned int current_max, unsigned int torque_max,
                unsigned int torque_slope, double prof_accel, double qs_decel); // current and torque in thousandths of rated current and torque

    void start();

    void autoSetup(); // Motor must be unloaded, not touched, and free to rotate in any direction.

    void setMode(uint16_t mode);

    // Torque in Nm
    void setTorque(double torque);

    // Speed in rad/s
    void setVelocity(double velocity);

    // position in rad
    void setPosition(double phi);

    // Torque in Nm
    double getTorque();

    // Speed in rad/s
    double getVelocity();

    // position in rad
    double getPosition();

    uint16_t getStatus();

//    // Speed in rad/s
//    double getTargetVelocity();
//
//    // position in rad
//    double getTargetPosition();

    void update();

    // State machine transitions
    bool shutdown();

    bool switchOn();

    bool disableVoltage();

    bool quickStop();

    bool disableOperation();

    bool enableOperation();

    bool enableOperationAfterQuickStop();

    bool faultReset();

private:
    CANOpenDevice *motor_dev;
    uint32_t data;
    unsigned int torque_max, current_max, torque_slope;
    uint32_t profile_acceleration, quick_stop_deceleration;
    BytesUnion outgoing{}, incoming{};
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_TORQUEMOTOR_H
