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
#define OP_HOMING               0x06U

class TorqueMotor {
public:
    TorqueMotor(CANRaw *can_line, uint16_t node_id, unsigned int current_max, unsigned int torque_max,
                unsigned int torque_slope, float prof_accel, float qs_decel, float prof_vel); // current and torque in thousandths of rated current and torque

    void start();

    void autoSetup(); // Motor must be unloaded, not touched, and free to rotate in any direction.

    void setMode(uint16_t mode);

    void calibrate();

    // Torque in Nm
    void setTorque(float torque);

    // Speed in rad/s
    void setVelocity(float velocity);

    // position in rad
    void setPosition(float phi);

    // Torque in Nm
    float getTorque();

    // Speed in rad/s
    float getVelocity();

    // position in rad
    float getPosition();

    uint16_t getStatus();

//    // Speed in rad/s
//    float getTargetVelocity();
//
//    // position in rad
//    float getTargetPosition();

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
    uint32_t profile_acceleration, quick_stop_deceleration, profile_velocity;
    uint32_t homing_offset,homing_method,homing_velocity,homing_acceleration,homing_current,homing_period;
    BytesUnion outgoing{}, incoming{};
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_TORQUEMOTOR_H
