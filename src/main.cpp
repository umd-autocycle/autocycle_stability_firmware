//
// Created by Cooper Grill on 3/31/2020.
//

// Arduino libraries
#include <Arduino.h>
#include <Wire.h>
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
#define TM_TORQUE_SLOPE     1000

// Constants
#define FTHRESH PI/4.0      // Threshold for being fallen over
#define UTHRESH PI/20.0


IMU imu(0x68);
Indicator indicator(3, 4, 5, 11);
TorqueMotor *torque_motor;
DriveMotor *drive_motor;

// State variables
uint8_t user_req = 0;       // User request binary flags
double phi = 0.0;
double del = 0.0;
double v = 0.0;


float currentSpeed = 0; //in m/s
float desiredSpeed; //in m/s

//void maintainStability() {
//    if (phi == NULL) {
//        phi = atan2(ay, az);  //initial conditions
//    }
//    delay(1);
//    phi = (0.98 * (phi * 180 / PI + gx * 0.001) + 0.02 * atan2(ay, az) * 180 / PI) * PI /
//          180;    //complementary filter to determine roll (in radians/s)
//
//    double e[] = {phi, 0, gx * PI / 180, 0};
//    double torque = get_torque(0.001, e, 5, 20.0);
//    Serial.print("Torque = "); Serial.println(torque);
//}


//
//void updateSpeed() {
//    delT2 = millis();
//    if (delT1 == 0) {
//        currentSpeed = 0;
//    } else {
//        currentSpeed = circumference/((delT2-delT1)/1000);
//    }
//    delT1 = delT2;
//}

void setup() {
    Wire.begin();                              // Begin I2C interface
    Serial.begin(115200);           // Begin Serial (UART to USB) communication
    Serial1.begin(1200);
    Serial2.begin(1200);
    delay(1000);
    //Can0.begin(CAN_BPS_1000K, 0xFF);     // 1M baud rate, no enable pin

    analogWriteResolution(12);        // Enable expanded PWM and ADC resolution
    analogReadResolution(12);

    // Initiate indicator
    indicator.start();
    indicator.beep(1);
    //indicator.cycle()
    indicator.setPassiveRGB(RGB_STARTUP_P);
    indicator.setBlinkRGB(RGB_STARTUP_B);
    indicator.silence();
    torque_motor = new TorqueMotor(&Can0, TM_NODE_ID, TM_CURRENT_MAX, TM_TORQUE_MAX, TM_TORQUE_SLOPE);
    //torque_motor->start();                          // Initialize torque control motor
    drive_motor = new DriveMotor(1); //1 = 1 m/s
    drive_motor->start();

//    imu.start();                                    // Initialize IMU
//    imu.configure(2, 2, 1);  // Set accelerometer and gyro resolution, on-chip low-pass filter
//
//    if (imu.calibrateGyros()) {
//        Serial.println("Gyroscopes Successfully Calibrated.");
//        indicator.beepstring((uint8_t) 0b01110111);
//    } else {
//        indicator.beepstring((uint16_t) 0b0000000100000001);
//    }
//
//    if (imu.calibrateAccel(0, 0, GRAV)) {
//        Serial.println("Accelerometers Successfully Calibrated.");
//        indicator.beepstring((uint8_t) 0b10101010);
//    } else {
//        indicator.beepstring((uint8_t) 0b10001000, 1);
//    }
//
//
//    indicator.setPassiveRGB(RGB_IDLE_P);
//    indicator.setBlinkRGB(RGB_IDLE_B);
}

void loop() {
    static uint8_t state = 0;

    if(Serial.available()) {
        char command = Serial.read();
        if (command == 'r') {
            drive_motor->resetMotor();
        }
        else if (command == 'c'){
            int current = Serial.parseInt(); //value between 0 and 100- be careful not to set it too high!
            drive_motor->setCurrent(current);
        }
        else if (command == 's'){
            Serial.println("in setspeed");
            int speed = Serial.parseInt(); //value between 0 and 100
            Serial.println(speed);
            drive_motor->setSpeed(speed);
        }
        else if (command == 'b')
        {
            drive_motor->storeBasic();
        }
        else if (command == 't')
        {
            drive_motor->storeThrottle();
        }
        else if (command == 'p'){
            Serial.println("in setpas");
            int speed = Serial.parseInt(); //value between 0 and 100
            Serial.println(speed);
            drive_motor->setPASNum(speed);
        }
    }

    // Update sensor information
    imu.update();
    torque_motor->update();

    // Update state variables

    // Update indicator
    indicator.update();

    //send info back and forth between display and motor
    if(Serial1.available())
    {
        drive_motor->readMotorSignal(false);
    }
    if(Serial2.available())
    {
        drive_motor->readDisplaySignal();
    }


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
            }
            break;

        case CALIB:     // Sensor calibration
            if (true) {
                user_req &= ~R_CALIB;
                state = IDLE;
                indicator.disablePulse();
                indicator.setPassiveRGB(RGB_IDLE_P);
                indicator.setBlinkRGB(RGB_IDLE_B);
            }
            break;

        case MANUAL:    // Manual operation
            if (user_req & R_RESUME) {
                user_req &= ~(R_RESUME | R_MANUAL);
                state = IDLE;
                indicator.setPassiveRGB(RGB_IDLE_P);
                indicator.setBlinkRGB(RGB_IDLE_B);
            }
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
            drive_motor->queryRPM();
            break;

        case FALLEN:    // Fallen

            // Transitions
            if (fabs(phi) < UTHRESH) {
                state = IDLE;
                indicator.setPassiveRGB(RGB_IDLE_P);
                indicator.setBlinkRGB(RGB_IDLE_B);
                indicator.disablePulse();
            }
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
            break;

        default:        // Invalid state, fatal error
            break;
    }
}