//
// Created by evanr on 9/1/2020.
//

#include "DriveMotor.h"
#include <Arduino.h>

DriveMotor::DriveMotor(int driveMotorPin, int speedSensorPin, float desiredSpeed) {
    this->driveMotorPin = driveMotorPin;
    this->speedSensorPin = speedSensorPin;
    pinMode(speedSensorPin, INPUT);
    pinMode(driveMotorPin, OUTPUT);
}

void DriveMotor::start(void (*func)(void)) {  //use [] {drivemotor.convertSignalToSpeed() in main}
    attachInterrupt(digitalPinToInterrupt(speedSensorPin), func, RISING);
}

void DriveMotor::convertSignalToSpeed() {
    delT2 = millis();
    if (delT1 == 0) {
        currentSpeed = 0;
    } else if ((delT2 - delT1) < 100) {
        return;
    } else {
        currentSpeed = circumference / ((delT2 - delT1) / 1000);
    }
    delT1 = delT2;
    writeAnalog();
}


void DriveMotor::writeAnalog() {
    analogWrite(driveMotorPin, maintainSpeed(desiredSpeed, currentSpeed));
}

int DriveMotor::maintainSpeed(float desiredSpeed, float currentSpeed) {
    float speedKp = .01;
    float speedKd = .01;
    float speedPreError = 0;
    float dt = .01;
    float outputMax = .5;
    float outputMin = -.5;

    //testing maintainSpeed
    //assume starting units are already in m/s, and matching output units (also in m/s)

    float speedError = desiredSpeed - currentSpeed;


    float pOut = speedKp * speedError;
    float speedDeriv = (speedError - speedPreError) / dt;
    float dOut = speedKd * speedDeriv;
    float output = dOut + pOut;

    if (output > outputMax)
        output = outputMax;
    else if (output < outputMin)
        output = outputMin;

    //currentSpeed += output; // this is just for testing, output will eventually actually send a signal
    speedPreError = speedError;
    speedError = desiredSpeed - currentSpeed;
    return int((currentSpeed += output) / (maxThrottle - minThrottle) * 4096); //what is due analog res?
}

void DriveMotor::readSpeedSignal() {

}


