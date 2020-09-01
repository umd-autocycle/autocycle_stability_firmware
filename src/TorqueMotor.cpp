//
// Created by Misha on 7/1/2020.
//

#include "TorqueMotor.h"
#include "CANOpen.h"
#include <due_can.h>

TorqueMotor::TorqueMotor(CANRaw *can_line, uint16_t node_id, unsigned int current_max, unsigned int torque_max,
                         unsigned int torque_slope) {
    motor_dev = new CANOpenDevice(can_line, node_id);

    this->current_max = current_max;
    this->torque_max = torque_max;
    this->torque_slope = torque_slope;
}

void TorqueMotor::start() {
    uint32_t data;
    motor_dev->readSDO(0x6041, 0, data);    // Read status word

    // Set motor data
    motor_dev->writeSDO(0x2030U, 0, SDO_WRITE_4B, POLE_PAIR_COUNT);
    motor_dev->writeSDO(0x2031U, 0, SDO_WRITE_4B, MAX_PERM_CURRENT_MA);
    motor_dev->writeSDO(0x6075U, 0, SDO_WRITE_2B, RATED_CURRENT_MA);
    motor_dev->writeSDO(0x6073U, 0, SDO_WRITE_2B, torque_max);
    motor_dev->writeSDO(0x203BU, 2, SDO_WRITE_4B, MAX_CURRENT_DUR_MS);
    motor_dev->writeSDO(0x3202U, 0, SDO_WRITE_4B, BLDC_MOTOR);

    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_2B, 0x0006);

}

void TorqueMotor::commission() {
    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_1B, OP_AUTO_SETUP);
    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_2B, 0x0007);

    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_2B, 0x000F);
    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_2B, 0x001F);

}

void TorqueMotor::torqueMode() {

}

void TorqueMotor::setTorque() {

}

void TorqueMotor::update() {

}



