//
// Created by evanr on 9/1/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
#define AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H

//goals of class:
//handle communication btwn due and drive motor
//control set speed
//receive input of digital beeps, convert to speed
//send analog output back to motor

class DriveMotor{
public:
    DriveMotor(int driveMotorPin,int speedSensorPin, float desiredSpeed);

    void readSpeedSignal();

    void convertSignalToSpeed();

    void writeAnalog();
private:
    float delT1 = 0;
    float delT2;
    float radius = .35; //dummy value in inches, need to measure
    float circumference = 2.0f * 3.14f * radius;
    int const interruptPin = 26;
    float currentSpeed = 0;
    float desiredSpeed;
    int driveMotorPin;
    int speedSensorPin;

};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
