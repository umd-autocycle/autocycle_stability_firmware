//
// Created by Misha on 7/1/2020.
//

#include "TorqueMotor.h"
#include <due_can.h>
#include <Arduino.h>

// Motor data
#define POLE_PAIR_COUNT         6
#define MAX_PERM_CURRENT_MA     16000
#define RATED_CURRENT_MA        5330
#define MAX_CURRENT_DUR_MS      100
#define BLDC_MOTOR              0x0000000041U
#define RATED_TORQUE_N          0.5f
#define GEARING                 33.0f
#define EFFICIENCY              0.93f

// Position control word flags
#define CTRW_POSITION_MOVE_COMMAND  0b0000000000010000U
#define CTRW_POSITION_IMMEDIATE     0b0000000000100000U
#define CTRW_POSITION_RELATIVE      0b0000000001000000U
#define CTRW_POSITION_HALT          0b0000000100000000U
#define CTRW_POSITION_NO_DWELL      0b0000001000000000U

// Position status word flags
#define STAW_POSITION_REACHED       0b0000010000000000U
#define STAW_POSITION_LIM_EXCEEDED  0b0000100000000000U
#define STAW_POSITION_NEW_SETPOINT  0b0001000000000000U
#define STAW_POSITION_FOLLOW_ERROR  0b0010000000000000U

// Velocity control word flags
#define CTRW_VELOCITY_HALT          0b0000000100000000U

// Velocity status word flags
#define STAW_VELOCITY_REACHED       0b0000010000000000U
#define STAW_VELOCITY_DEV_ERROR     0b0010000000000000U

#define TORQUE_RX_PDO_NUM 0
#define TORQUE_TX_PDO_NUM 0
#define VELOCITY_RX_PDO_NUM 1
#define VELOCITY_TX_PDO_NUM 1
#define POSITION_RX_PDO_NUM 2
#define POSITION_TX_PDO_NUM 2
#define CONTROL_RX_PDO_NUM 3
#define CONTROL_TX_PDO_NUM 3

TorqueMotor::TorqueMotor(CANRaw *can_line, uint16_t node_id, unsigned int current_max, unsigned int torque_max,
                         unsigned int torque_slope, float prof_accel, float qs_decel, float prof_vel) {
    motor_dev = new CANOpenDevice(can_line, node_id);

    this->current_max = current_max;
    this->torque_max = torque_max;
    this->torque_slope = torque_slope;

    outgoing.value = 0;
    profile_acceleration = prof_accel * 100 * GEARING;
    quick_stop_deceleration = qs_decel * 100 * GEARING;
    profile_velocity = prof_vel * 100 * GEARING;
}

void TorqueMotor::start() {
    data = 0;

    // Reset communications
    motor_dev->networkCommand(0x81);
    motor_dev->waitForBoot();
    delay(5000);

//    while (!disableVoltage());

    // Set motor data
    motor_dev->writeSDO(0x2030U, 0, SDO_WRITE_4B, POLE_PAIR_COUNT);
    motor_dev->writeSDO(0x2031U, 0, SDO_WRITE_4B, MAX_PERM_CURRENT_MA);
    motor_dev->writeSDO(0x203BU, 1, SDO_WRITE_4B, RATED_CURRENT_MA);    // Set rated current
    motor_dev->writeSDO(0x203BU, 2, SDO_WRITE_4B, MAX_CURRENT_DUR_MS);
    motor_dev->writeSDO(0x3202U, 0, SDO_WRITE_4B, BLDC_MOTOR); // with closed loop operation         // Set motor type

    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_1B, OP_NONE);             // Set operation mode to none
    motor_dev->writeSDO(0x6073U, 0, SDO_WRITE_2B, current_max);         // Set tenth percents of rated current allowed

    motor_dev->writeSDO(0x60A8U, 0, SDO_WRITE_4B,
                        (0xFEU << 24U) | (0x10U << 16U)); // Set units of position to tenths of a radian
    motor_dev->writeSDO(0x60A9U, 0, SDO_WRITE_4B, (0xFEU << 24U) | (0x10U << 16U) | (0x03U
            << 8U)); // Set units of velocity to tenths of a radian per second

    motor_dev->writeSDO(0x6080U, 0, SDO_WRITE_4B, 47124); // Max motor speed (.01 rad/s)
    motor_dev->writeSDO(0x607FU, 0, SDO_WRITE_4B, 47124); // Max profile velocity

    // Set up RX PDOs needed for torque mode
    PDOMapping rx_torque[1];
    rx_torque[0] = {0x6071U, 0, 16}; // Target torque
    motor_dev->configureRxPDO(TORQUE_RX_PDO_NUM, PDO_RX_TRANS_ASYNC, 1, rx_torque);

    // Set up TX PDOs needed for torque mode
    PDOMapping tx_torque[1];
//    tx_torque[0] = {0x6074U, 0, 16}; // Current torque demand
    tx_torque[0] = {0x6077U, 0, 16}; // Current actual torque
    motor_dev->configureTxPDO(TORQUE_TX_PDO_NUM, PDO_TX_TRANS_ASYNC_TIME, 100, 20, 1, tx_torque);


    // Set up RX PDOs needed for velocity mode
    PDOMapping rx_velocity[1];
    rx_velocity[0] = {0x60FFU, 0, 32}; // Target velocity
    motor_dev->configureRxPDO(VELOCITY_RX_PDO_NUM, PDO_RX_TRANS_ASYNC, 1, rx_velocity);

    // Set up TX PDOs needed for velocity mode
    PDOMapping tx_velocity[1];
//    tx_velocity[0] = {0x606BU, 0, 32}; // Current velocity demand
    tx_velocity[0] = {0x606CU, 0, 32}; // Current actual velocity
    motor_dev->configureTxPDO(VELOCITY_TX_PDO_NUM, PDO_TX_TRANS_ASYNC_TIME, 100, 20, 1, tx_velocity);


    // Set up RX PDOs needed for position mode
    PDOMapping rx_position[1];
    rx_position[0] = {0x607AU, 0, 32}; // Target position
    motor_dev->configureRxPDO(POSITION_RX_PDO_NUM, PDO_RX_TRANS_ASYNC, 1, rx_position);

    // Set up TX PDOs needed for position mode
    PDOMapping tx_position[1];
//    tx_position[0] = {0x6062U, 0, 32}; // Current position demand
    tx_position[0] = {0x6064U, 0, 32}; // Current actual position
    motor_dev->configureTxPDO(POSITION_TX_PDO_NUM, PDO_TX_TRANS_ASYNC_TIME, 100, 20, 1, tx_position);

    // Set up RX PDOs needed for control mode
    PDOMapping rx_control[1];
    rx_control[0] = {0x6040U, 0, 16}; // Control word
    motor_dev->configureRxPDO(CONTROL_RX_PDO_NUM, PDO_RX_TRANS_ASYNC, 1, rx_control);

    // Set up TX PDOs needed for control mode
    PDOMapping tx_control[1];
    tx_control[0] = {0x6041U, 0, 16}; // Current control demand
    motor_dev->configureTxPDO(CONTROL_TX_PDO_NUM, PDO_TX_TRANS_ASYNC_TIME, 100, 20, 1, tx_control);

    delay(1000);


    // Set network mode to operational
    motor_dev->networkCommand(0x01);
    delay(1000);


    while (!shutdown());
}

void TorqueMotor::autoSetup() {
    uint32_t status = 0;

    shutdown();

    // Set motor mode to Auto Setup
    motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_1B, OP_AUTO_SETUP);

    switchOn();
    enableOperation();

    // Begin auto setup
    motor_dev->writeSDO(0x6040U, 0, SDO_WRITE_2B, 0x001F);

    // Wait until auto setup is complete
    while ((status & 0x1237U) != 0x1237U) {
        motor_dev->readSDO(0x6041U, 0, status);
        delay(100);
    }
    status = 0;

    while (!shutdown());


    // Start saving tuning parameters
    motor_dev->writeSDO(0x1010U, 0x06U, SDO_WRITE_4B, 0x65766173U); // write "save"

    // Wait until tuning parameters are saved
    while (status != 1) {
        motor_dev->readSDO(0x1010U, 0x06U, status);
        delay(100);
    }
    status = 0;

    // Start saving drive parameters
    motor_dev->writeSDO(0x1010U, 0x05U, SDO_WRITE_4B, 0x65766173U); // write "save"

    // Wait until drive parameters are saved
    while (status != 1) {
        motor_dev->readSDO(0x1010U, 0x05U, status);
        delay(100);
    }
    status = 0;
}

void TorqueMotor::setMode(uint16_t mode) {
    while (!shutdown());
    uint32_t submode_select = 0;

    switch (mode) {
        case OP_PROFILE_TORQUE:
            // Set operation mode to profile torque
            motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_1B, OP_PROFILE_TORQUE);

            // Set submode to "real torque" mode
            submode_select = 0b00000000000000000000000001100001;
            motor_dev->writeSDO(0x3202U, 0, SDO_WRITE_4B, submode_select);
            motor_dev->readSDO(0x3202U, 0, submode_select);

            motor_dev->writeSDO(0x6072U, 0, SDO_WRITE_2B, torque_max); // Max torque
            motor_dev->writeSDO(0x6087U, 0, SDO_WRITE_4B, torque_slope); // Torque slope
            break;

        case OP_PROFILE_VELOCITY:
            // Set operation mode to profile velocity
            motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_1B, OP_PROFILE_VELOCITY);

            // Configure velocity curve parameters
            motor_dev->writeSDO(0x6083U, 0, SDO_WRITE_4B,
                                profile_acceleration);  // Set profile acceleration and deceleration
            motor_dev->writeSDO(0x6084U, 0, SDO_WRITE_4B, profile_acceleration);
            motor_dev->writeSDO(0x6084U, 0, SDO_WRITE_4B, quick_stop_deceleration);
            motor_dev->writeSDO(0x6086U, 0, SDO_WRITE_2B, 3);   // Set motion to jerk-limited ramp
            motor_dev->writeSDO(0x60C5U, 0, SDO_WRITE_4B, 2 * profile_acceleration);
            motor_dev->writeSDO(0x60C6U, 0, SDO_WRITE_4B, 2 * quick_stop_deceleration);

            break;

        case OP_PROFILE_POSITION:
            motor_dev->writeSDO(0x6060U, 0, SDO_WRITE_1B, OP_PROFILE_POSITION);

            // Configure velocity curve parameters
            motor_dev->writeSDO(0x6083U, 0, SDO_WRITE_4B,
                                profile_acceleration);  // Set profile acceleration and deceleration
            motor_dev->writeSDO(0x6084U, 0, SDO_WRITE_4B, profile_acceleration);
            motor_dev->writeSDO(0x6084U, 0, SDO_WRITE_4B, quick_stop_deceleration);
            motor_dev->writeSDO(0x6086U, 0, SDO_WRITE_2B, 0);   // Set motion to not jerk-limited ramp
            motor_dev->writeSDO(0x60C5U, 0, SDO_WRITE_4B, 2 * profile_acceleration);
            motor_dev->writeSDO(0x60C6U, 0, SDO_WRITE_4B, 2 * quick_stop_deceleration);

            // Configure position curve parameters
            motor_dev->writeSDO(0x6081U, 0, SDO_WRITE_4B, profile_velocity);
            motor_dev->writeSDO(0x6082U, 0, SDO_WRITE_4B, 0);

            // Have controller immediately reset the new setpoint bit
            motor_dev->writeSDO(0x60F2U, 0, SDO_WRITE_2B, 0x0021U);

            break;

        default:
            break;
    }

    while (!switchOn());
}

void TorqueMotor::setTorque(float torque) {
    int16_t torque_thou = 1000.0 * torque / GEARING / EFFICIENCY / RATED_TORQUE_N;
    outgoing.s0 = torque_thou;
    outgoing.s1 = 0;
    outgoing.s2 = 0;
    outgoing.s3 = 0;
    motor_dev->writePDO(TORQUE_RX_PDO_NUM, outgoing);
}

void TorqueMotor::setVelocity(float velocity) {
    int32_t desired_velocity = 100 * velocity * GEARING;
    outgoing.low = desired_velocity;
    outgoing.high = 0;
    motor_dev->writePDO(VELOCITY_RX_PDO_NUM, outgoing);
}

void TorqueMotor::setPosition(float phi) {
    int32_t desired_position = 100 * phi * GEARING;
    outgoing.low = desired_position;
    outgoing.high = 0;
    motor_dev->writePDO(POSITION_RX_PDO_NUM, outgoing);

    outgoing.s0 = CTRW_POSITION_IMMEDIATE | CTRW_POSITION_MOVE_COMMAND | 0b0000000000001111U;
    outgoing.s1 = 0;
    outgoing.s2 = 0;
    outgoing.s3 = 0;
    motor_dev->writePDO(CONTROL_RX_PDO_NUM, outgoing);
}

float TorqueMotor::getTorque() {
    motor_dev->readPDO(TORQUE_TX_PDO_NUM, incoming);
    int16_t torque_thou = incoming.s0;

    return (1 / 1000.0f) * GEARING * EFFICIENCY * RATED_TORQUE_N * (float) torque_thou;
}

float TorqueMotor::getVelocity() {
    motor_dev->readPDO(VELOCITY_TX_PDO_NUM, incoming);

    return ((int32_t) incoming.low) / 100.0f / GEARING;
}

float TorqueMotor::getPosition() {
    motor_dev->readPDO(POSITION_TX_PDO_NUM, incoming);

    return ((int32_t) incoming.low) / 100.0f / GEARING;
}

uint16_t TorqueMotor::getStatus() {
    motor_dev->readPDO(CONTROL_TX_PDO_NUM, incoming);

    return incoming.s0;
}

//float TorqueMotor::getTargetVelocity() {
//    motor_dev->readPDO(VELOCITY_TX_PDO_NUM, incoming);
//
//    return ((int32_t) incoming.low) / 100.0 / GEARING;
//}
//
//float TorqueMotor::getTargetPosition() {
//    motor_dev->readPDO(POSITION_TX_PDO_NUM, incoming);
//
//    return ((int32_t) incoming.low) / 100.0 / GEARING;
//}

void TorqueMotor::update() {
    motor_dev->update();
}

bool TorqueMotor::shutdown() {
    uint32_t status = 0;

    // Shutdown
    motor_dev->writeSDO(0x6040U, 0, SDO_WRITE_2B, 0b00000110);

    // Check if successful
    motor_dev->readSDO(0x6041U, 0, status);

    return (status & 0b00100001U) == 0b00100001U;
}

bool TorqueMotor::switchOn() {
    uint32_t status = 0;

    // Switch On
    motor_dev->writeSDO(0x6040U, 0, SDO_WRITE_2B, 0b00000111);

    // Check if successful
    motor_dev->readSDO(0x6041U, 0, status);

    return (status & 0b00100011U) == 0b00100011U;
}

bool TorqueMotor::disableVoltage() {
    uint32_t status = 0;

    // Disable Voltage
    motor_dev->writeSDO(0x6040U, 0, SDO_WRITE_2B, 0b00000000);

    // Check if successful
    motor_dev->readSDO(0x6041U, 0, status);

    return (status & 0b01000000U) == 0b01000000U;
}

bool TorqueMotor::quickStop() {
    uint32_t status = 0;

    // Quick Stop
    motor_dev->writeSDO(0x6040U, 0, SDO_WRITE_2B, 0b00000010);

    // Check if successful
    motor_dev->readSDO(0x6041U, 0, status);

    return (status & 0b00000111U) == 0b00000111U;
}

bool TorqueMotor::disableOperation() {
    uint32_t status = 0;

    // Disable Operation
    motor_dev->writeSDO(0x6040U, 0, SDO_WRITE_2B, 0b00000111);

    // Check if successful
    motor_dev->readSDO(0x6041U, 0, status);

    return (status & 0b00100011U) == 0b00100011U;
}

bool TorqueMotor::enableOperation() {
    uint32_t status = 0;

    // Enable Operation
    motor_dev->writeSDO(0x6040U, 0, SDO_WRITE_2B, 0b00001111);

    // Check if successful
    motor_dev->readSDO(0x6041U, 0, status);

    return (status & 0b00100111U) == 0b00100111U;
}

bool TorqueMotor::enableOperationAfterQuickStop() {
    uint32_t status = 0;

    // Enable Operation after Quick Stop
    motor_dev->writeSDO(0x6040U, 0, SDO_WRITE_2B, 0b00001111);

    // Check if successful
    motor_dev->readSDO(0x6041U, 0, status);

    return (status & 0b00100111U) == 0b00100111U;
}

bool TorqueMotor::faultReset() {
    uint32_t status = 0;

    // Reset from fault
    motor_dev->writeSDO(0x6040U, 0, SDO_WRITE_2B, 0b10000000);

    // Check if successful
    motor_dev->readSDO(0x6041U, 0, status);

    return (status & 0b01000000U) == 0b01000000U;
}

