//
// Created by evanr on 9/1/2020.
//

#include "DriveMotor.h"
#include <Arduino.h>

DriveMotor::DriveMotor(int driveMotorPin, int speedSensorPin, float desiredSpeed,void (*func) (void) ) { //use [] {drivemotor.convertSignalToSpeed() in main}
    this->driveMotorPin = driveMotorPin;
    this->speedSensorPin = speedSensorPin;
    pinMode(speedSensorPin, INPUT);
    pinMode(driveMotorPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(speedSensorPin), func, RISING);
}

void DriveMotor::convertSignalToSpeed() {
    delT2 = millis();
    if (delT1 == 0) {
        currentSpeed = 0;
    }
    else if ((delT2-delT1) < 100) {
        return;
    }
    else {
            currentSpeed = circumference / ((delT2 - delT1) / 1000);
        }
    }
    delT1 = delT2;
    writeAnalog();
}


void DriveMotor::writeAnalog() {
    analogWrite(driveMotorPin, maintainSpeed(desiredSpeed, currentSpeed));
}