//
// Created by evanr on 9/1/2020.
//

#include "DriveMotor.h"

DriveMotor::DriveMotor(int driveMotorPin, int speedSensorPin, float desiredSpeed)
{
    this->driveMotorPin = driveMotorPin;
    this->speedSensorPin = speedSensorPin;
    pinMode(speedSensorPin, INPUT);
    pinMode(driveMotorPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(speedSensorPin), DriveMotor::convertSignalToSpeed(), RISING);
}

void DriveMotor::convertSignalToSpeed()
{
    delT2 = millis();
    if (delT1 == 0) {
        currentSpeed = 0;
    } else {
        currentSpeed = circumference/((delT2-delT1)/1000);
    }
    delT1 = delT2;
    DriveMotor::writeAnalog();
}


void DriveMotor::writeAnalog()
{
    analogWrite(driveMotorPin, maintainSpeed(desiredSpeed,currentSpeed));
}