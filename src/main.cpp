//
// Created by Cooper Grill on 3/31/2020.
//

// Arduino libraries
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <due_can.h>

// Internal libraries
#include "IMU.h"
#include "Indicator.h"
#include "controls.h"
#include "CANOpen.h"
#include "TorqueMotor.h"
#include "DriveMotor.h"

// States
#define IDLE    0
#define CALIB   1
#define MANUAL  2
#define ASSIST  3
#define AUTO    4
#define FALLEN  5
#define E_STOP  6

// State colors
#define RGB_STARTUP_P   255, 255, 255
#define RGB_IDLE_P      255, 255, 0
#define RGB_CALIB_P     128, 0, 128
#define RGB_MANUAL_P    255, 165, 0
#define RGB_ASSIST_P    34, 139, 34
#define RGB_AUTO_P      0, 255, 0
#define RGB_FALLEN_P    255, 140, 0
#define RGB_E_STOP_P    255, 0, 0
#define RGB_STARTUP_B   0, 0, 255
#define RGB_IDLE_B      0, 0, 255
#define RGB_CALIB_B     128, 255, 128
#define RGB_MANUAL_B    0, 89, 255
#define RGB_ASSIST_B    140, 34, 140
#define RGB_AUTO_B      255, 0, 255
#define RGB_FALLEN_B    255, 0, 0
#define RGB_E_STOP_B    0, 0, 255


// User request bit flags
#define R_CALIB     0b00000001U
#define R_MANUAL    0b00000010U
#define R_STOP      0b00000100U
#define R_RESUME    0b00001000U

// Info for torque motor
#define TM_NODE_ID          127
#define TM_CURRENT_MAX      1000
#define TM_TORQUE_MAX       1000
#define TM_TORQUE_SLOPE     10000   // Thousandths of max torque per second

// State transition constants
#define FTHRESH PI/4.0      // Threshold for being fallen over
#define UTHRESH PI/20.0     // Threshold for being back upright


// Device object definitions
IMU imu(0x68);
Indicator indicator(3, 4, 5, 11);
TorqueMotor *torque_motor;
DriveMotor *drive_motor;


// State variables
uint8_t user_req = 0;       // User request binary flags
double phi = 0.0;           // Roll angle (rad)
double del = 0.0;           // Steering angle (rad)
double dphi = 0.0;          // Roll angle rate (rad/s)
double ddel = 0.0;          // Steering angle rate (rad/s)
double v = 0.0;             // Velocity (m/s)

// Reference variables
double phi_r = 0.0;         // Required roll angle (rad)
double del_r = 0.0;         // Required steering angle (rad)
double v_r = 0.0;           // Required velocity (m/s)

// Control variables
double torque = 0.0;        // Current torque (Nm)


// State actions
void idle();

void calibrate();

void manual();

void assist();

void automatic();

void fallen();

void emergency_stop();

void report(uint8_t state);


void setup() {
    Wire.begin();                               // Begin I2C interface
    SPI.begin();                                // Begin Serial Peripheral Interface (SPI)

    Serial.begin(115200);             // Begin Main Serial (UART to USB) communication
    Serial1.begin(1200);              // Begin Bafang Serial (UART) communication
    Serial2.begin(1200);
    delay(1000);                          // Wait for Serial interfaces to initialize

    Can0.begin(CAN_BPS_1000K);                  // Begin 1M baud rate CAN interface, no enable pin
    Can0.watchFor();                            // Watch for all incoming CANbus messages

    analogWriteResolution(12);              // Enable expanded PWM and ADC resolution
    analogReadResolution(12);

    // Initialize indicator
    indicator.start();
    indicator.beep(100);
    indicator.setPassiveRGB(RGB_STARTUP_P);
    indicator.setBlinkRGB(RGB_STARTUP_B);

    // Initialize torque control motor
    torque_motor = new TorqueMotor(&Can0, TM_NODE_ID, TM_CURRENT_MAX, TM_TORQUE_MAX, TM_TORQUE_SLOPE,
                                   8 * PI, 16 * PI, 10);
    torque_motor->start();

    // Initialize Bafang drive motor
    drive_motor = new DriveMotor(DAC0);
    drive_motor->start();

    imu.start();                                    // Initialize IMU
    imu.configure(2, 2, 1);  // Set accelerometer and gyro resolution, on-chip low-pass filter

    indicator.setPassiveRGB(RGB_IDLE_P);
    indicator.setBlinkRGB(RGB_IDLE_B);
}

void loop() {
    static uint8_t state = IDLE;

    // Update sensor information
    imu.update();
    torque_motor->update();

    // Update state variables
    phi = atan2(imu.accelY(), imu.accelZ());    // TODO add Kalman filter
    dphi = imu.gyroX();                         // TODO add Kalman filter
    del = torque_motor->getPosition();
    ddel = torque_motor->getVelocity();
    torque = torque_motor->getTorque();
    v = drive_motor->getSpeed();                // TODO add Kalman filter

    // Update indicator
    indicator.update();


    // Act based on machine state, transition if necessary
    switch (state) {
        case IDLE:      // Idle
            // Transitions
            if (fabs(phi) > FTHRESH) {
                state = FALLEN;
                indicator.setPassiveRGB(RGB_FALLEN_P);
                indicator.setBlinkRGB(RGB_FALLEN_B);
                indicator.setPulse(500, 1500);
            }
            if (v > 1.0) {
                state = ASSIST;
                indicator.setPassiveRGB(RGB_ASSIST_P);
                indicator.setBlinkRGB(RGB_ASSIST_B);
            }
            if (user_req & R_CALIB) {
                state = CALIB;
                indicator.setPassiveRGB(RGB_CALIB_P);
                indicator.setBlinkRGB(RGB_CALIB_B);
                indicator.setPulse(250, 250);
            }
            if (user_req & R_MANUAL) {
                state = MANUAL;
                indicator.setPassiveRGB(RGB_MANUAL_P);
                indicator.setBlinkRGB(RGB_MANUAL_B);

                torque_motor->setMode(OP_PROFILE_POSITION);
                while (!torque_motor->enableOperation());
            }

            // Action
            idle();

            break;

        case CALIB:     // Sensor calibration
            // Transitions
            if (true) {
                user_req &= ~R_CALIB;
                state = IDLE;
                indicator.disablePulse();
                indicator.setPassiveRGB(RGB_IDLE_P);
                indicator.setBlinkRGB(RGB_IDLE_B);
            }

            // Action
            calibrate();

            break;

        case MANUAL:    // Manual operation
            // Transitions
            if (user_req & R_RESUME) {
                user_req &= ~(R_RESUME | R_MANUAL);
                state = IDLE;
                indicator.setPassiveRGB(RGB_IDLE_P);
                indicator.setBlinkRGB(RGB_IDLE_B);
            }

            // Action
            manual();

            break;

        case ASSIST:    // Assisted (training wheel) motion, only control heading
            // Transitions
            if (fabs(phi) > FTHRESH) {
                state = FALLEN;
                indicator.setPassiveRGB(RGB_FALLEN_P);
                indicator.setBlinkRGB(RGB_FALLEN_B);
                indicator.setPulse(500, 1500);
            }
            if (v > 4.0) {
                state = AUTO;
                indicator.setPassiveRGB(RGB_AUTO_P);
                indicator.setBlinkRGB(RGB_AUTO_B);
            }
            if (v < 0.5) {
                state = IDLE;
                indicator.setPassiveRGB(RGB_IDLE_P);
                indicator.setBlinkRGB(RGB_IDLE_B);
            }
            if (user_req & R_STOP) {
                state = E_STOP;
                indicator.setPassiveRGB(RGB_E_STOP_P);
                indicator.setBlinkRGB(RGB_E_STOP_B);
            }

            // Action
            assist();

            break;

        case AUTO:      // Automatic motion, control heading and stability
            // Transitions
            if (fabs(phi) > FTHRESH) {
                state = FALLEN;
                indicator.setPassiveRGB(RGB_FALLEN_P);
                indicator.setBlinkRGB(RGB_FALLEN_B);
                indicator.setPulse(500, 1500);
            }
            if (v < 3.5) {
                state = ASSIST;
                indicator.setPassiveRGB(RGB_ASSIST_P);
                indicator.setBlinkRGB(RGB_ASSIST_B);
            }
            if (user_req & R_STOP) {
                state = E_STOP;
                indicator.setPassiveRGB(RGB_E_STOP_P);
                indicator.setBlinkRGB(RGB_E_STOP_B);
            }

            // Action
            automatic();

            break;

        case FALLEN:    // Fallen
            // Transitions
            if (fabs(phi) < UTHRESH) {
                state = IDLE;
                indicator.setPassiveRGB(RGB_IDLE_P);
                indicator.setBlinkRGB(RGB_IDLE_B);
                indicator.disablePulse();
            }

            // Action
            fallen();

            break;

        case E_STOP:    // Emergency stop
            // Transitions
            if (fabs(phi) > FTHRESH) {
                state = FALLEN;
                indicator.setPassiveRGB(RGB_FALLEN_P);
                indicator.setBlinkRGB(RGB_FALLEN_B);
                indicator.setPulse(500, 1500);
            }
            if (user_req & R_RESUME) {
                user_req &= ~(R_RESUME | R_STOP);
                state = IDLE;
                indicator.setPassiveRGB(RGB_IDLE_P);
                indicator.setBlinkRGB(RGB_IDLE_B);
            }

            // Action
            emergency_stop();

            break;

        default:        // Invalid state, fatal error
            indicator.setPassiveRGB(0, 0, 0);
            break;
    }

    // Report state, reference, and control values
    report(state);

    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 's':
                drive_motor->setSpeed(Serial.parseFloat());
                break;
            default:
                break;
        }
    }
}

void idle() {

}

void calibrate() {
    if (imu.calibrateGyros()) {
        Serial.println("Gyroscopes Successfully Calibrated.");
        indicator.beepstring((uint8_t) 0b01110111);
    } else {
        indicator.beepstring((uint16_t) 0b0000000100000001);
    }

    if (imu.calibrateAccel(0, 0, GRAV)) {
        Serial.println("Accelerometers Successfully Calibrated.");
        indicator.beepstring((uint8_t) 0b10101010);
    } else {
        indicator.beepstring((uint8_t) 0b10001000, 1);
    }
}

void manual() {
    torque_motor->setPosition(del_r);
    drive_motor->setSpeed(v_r);
}

void assist() {
    torque_motor->setPosition(del_r);
    drive_motor->setSpeed(v_r);
}

void automatic() {
    // TODO calculate control signal, apply to torque and drive
}

void fallen() {
    // TODO respond to falling over
}

void emergency_stop() {
    // TODO implement braking for emergency stop
}

void report(uint8_t state) {
    Serial.print(state);
    Serial.print('\t');
    Serial.print(phi);
    Serial.print('\t');
    Serial.print(del);
    Serial.print('\t');
    Serial.print(dphi);
    Serial.print('\t');
    Serial.print(ddel);
    Serial.print('\t');
    Serial.print(v);
    Serial.print('\t');
    Serial.print(torque);
    Serial.print('\t');
    Serial.print(phi_r);
    Serial.print('\t');
    Serial.print(del_r);
    Serial.print('\t');
    Serial.print(v_r);
    Serial.println();
    Serial.flush();
}
