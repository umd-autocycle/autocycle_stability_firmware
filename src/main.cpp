//
// Created by Cooper Grill on 3/31/2020.
//

#include <Arduino.h>
#include <Wire.h>
#include <Scheduler.h>


#include "IMU.h"
#include "Indicator.h"
#include "controls.h"

//CAN libraries
//#include <CAN_Acquisition.h>
//#include <DueTimer.h>
//#include <OBD2.h>
//#include <due_can.h>
//#include <sn65hvd234.h>

IMU imu(0x68);
Indicator indicator(3, 4, 5, 11);

float currentSpeed = 0; //in m/s
float desiredSpeed; //in m/s
double phi = 0.0;
int maxThrottle = 7;    //in m/s, need to determine what max throttle actually corresponds to
int minThrottle = 0;
int const throttlePin = 0;  //this is the pin that will control throttle/speed output, to be determined which one for sure

//for speed
float delT1 = 0;
float delT2;
float radius = .35; //dummy value in inches, need to measure
float circumference = 2 * 3.14 * radius;
int const interruptPin = 26;

//receive input from imu
//convert python PID interpolated to C++
//send torque info to motor
//void maintainStability() {
//    if (phi == NULL) {
//        phi = atan2(ay, az);  //initial conditions
//    }
//    delay(1);
//    phi = (0.98 * (phi * 180 / PI + gx * 0.001) + 0.02 * atan2(ay, az) * 180 / PI) * PI /
//          180;    //complementary filter to determine roll (in radians/s)
//    Serial.print("Roll = ");
//    Serial.println(phi * 180 / PI);
//
//    double e[] = {phi, 0, gx * PI / 180, 0};
//    double torque = get_torque(0.001, e, 5, 20.0);
//    Serial.print("Torque = "); Serial.println(torque);
//}

//void maintainSpeed() {
//    float speedKp = .01;
//    float speedKd = .01;
//    float speedPreError = 0;
//    float dt = .01;
//    float outputMax = .5;
//    float outputMin = -.5;
//
//    //testing maintainSpeed
//    //assume starting units are already in m/s, and matching output units (also in m/s)
//
//    float speedError = desiredSpeed - currentSpeed;
//
//
//    float pOut = speedKp * speedError;
//    float speedDeriv = (speedError - speedPreError) / dt;
//    float dOut = speedKd * speedDeriv;
//    float output = dOut + pOut;
//
//    if (output > outputMax)
//        output = outputMax;
//    else if (output < outputMin)
//        output = outputMin;
//
//    //currentSpeed += output; // this is just for testing, output will eventually actually send a signal
//    speedPreError = speedError;
//    speedError = desiredSpeed - currentSpeed;
//    int newThrottle = int((currentSpeed += output)/(maxThrottle-minThrottle)*4096); //what is due analog res?
//    analogWrite(throttlePin, newThrottle);
//
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
    Wire.begin();                         // Begin I2C interface
    Serial.begin(115200);       // Begin Serial (UART to USB) communication

    analogWriteResolution(10);        // Enable expanded PWM and ADC resolution
    analogReadResolution(12);

    indicator.start();
    indicator.beep(5000);
    indicator.cycle();
    indicator.setRGB(0, 0, 0);

    imu.start();                                    // Initialize IMU
    imu.configure(2, 2, 1);  // Set accelerometer and gyro resolution, on-chip low-pass filter
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


    //speed stuff
//    analogWriteResolution(12);
//    //speedCounter = maxSpeedCounter;
//    circumference = 2 * 3.14 * radius;
//    pinMode(interruptPin, INPUT);
//    pinMode(throttlePin, OUTPUT);
//    attachInterrupt(digitalPinToInterrupt(interruptPin), updateSpeed, RISING);
//    //anytime the speed pin goes from low to high, this interrupt should update speed accordingly
//
//    Scheduler.startLoop(maintainStability);
    //Scheduler.startLoop(maintainSpeed);



}

void loop() {
    imu.update();
    Serial.print(imu.gyroX());
    Serial.print('\t');
    Serial.print(imu.gyroY());
    Serial.print('\t');
    Serial.print(imu.gyroZ());
    Serial.print('\t');
    Serial.print(imu.accelX());
    Serial.print('\t');
    Serial.print(imu.accelY());
    Serial.print('\t');
    Serial.print(imu.accelZ());
    Serial.println();
    delay(10);
}