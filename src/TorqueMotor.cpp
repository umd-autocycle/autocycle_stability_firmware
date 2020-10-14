//
// Created by Misha on 7/1/2020.
//

#include "TorqueMotor.h"
#include "CANOpen.h"
#include <due_can.h>

#define TORQUE_RX_PDO_NUM 0
#define TORQUE_TX_PDO_NUM 0

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
    motor_dev->writeSDO(0x203BU, 1, SDO_WRITE_2B, RATED_CURRENT_MA); // Set rated current
    motor_dev->writeSDO(0x203BU, 2, SDO_WRITE_4B, MAX_CURRENT_DUR_MS);
    motor_dev->writeSDO(0x3202U, 0, SDO_WRITE_4B, BLDC_MOTOR);       // Set motor type

    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_1B, 0);           // Set operation mode to none
    motor_dev->writeSDO(0x6073U, 0, SDO_WRITE_2B, current_max);         // Set tenth percents of rated current allowed


}

void TorqueMotor::commission() {
    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_1B, OP_AUTO_SETUP);
    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_2B, 0x0007);

    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_2B, 0x000F);
    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_2B, 0x001F);

}

void TorqueMotor::torqueMode() {
    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_1B, OP_PROFILE_TORQUE);

    // Set submode to "real torque" mode
    uint32_t submode_select = 0;
    motor_dev->readSDO(0x3202U, 0, submode_select);
    submode_select |= (uint32_t) (1U << 5U);
    motor_dev->writeSDO(0x3202U, 0, SDO_WRITE_4B, submode_select);

    // Set up RX PDOs needed for torque mode
    PDOMapping rx_torque[3];
    rx_torque[0] = {0x6071U, 0, 16}; // Target torque
    rx_torque[1] = {0x6072U, 0, 16}; // Max torque
    rx_torque[2] = {0x6087U, 0, 16}; // Torque slope
    motor_dev->configureRxPDO(TORQUE_RX_PDO_NUM, PDO_RX_TRANS_ASYNC, 3, rx_torque);

    // Set up TX PDOs needed for torque mode
    PDOMapping tx_torque[2];
    tx_torque[0] = {0x6074U, 0, 16}; // Current torque demand
    tx_torque[1] = {0x6077U, 0, 16}; // Current actual torque
    motor_dev->configureTxPDO(TORQUE_TX_PDO_NUM, PDO_TX_TRANS_ASYNC, 100, 100, 2, tx_torque);

}

void TorqueMotor::setTorque(double torque) {
    desired_torque = torque;
    uint16_t torque_thou = 1000.0 * (torque / GEARING / EFFICIENCY / RATED_TORQUE_N);
    outgoing.s0 = torque_thou;
    outgoing.s1 = torque_max;
    outgoing.s2 = torque_slope;
    outgoing.s3 = 0;
    motor_dev->writePDO(TORQUE_RX_PDO_NUM, outgoing);
}

void TorqueMotor::update() {
    motor_dev->update();
    actual_torque = GEARING * EFFICIENCY * RATED_TORQUE_N * motor_dev->tx_pdo_buffer[TORQUE_TX_PDO_NUM].s1;

}



