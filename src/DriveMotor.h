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

class DriveMotor {
public:
    DriveMotor(int driveMotorPin, int speedSensorPin, float desiredSpeed);

    void start(void (*func)(void));

    void readSpeedSignal();

    void convertSignalToSpeed();

    void writeAnalog();

    int maintainSpeed(float desiredSpeed, float currentSpeed);


private:
    float delT1 = 0;
    float delT2;
    const float radius = .35; //dummy value in inches, need to measure //const or preprocessor macros
    const float circumference = 2.0f * 3.14f * radius;//const or preprocessor macros
    float currentSpeed = 0;
    float desiredSpeed;
    int driveMotorPin;
    int speedSensorPin;
    int maxThrottle = 7;    //in m/s, need to determine what max throttle actually corresponds to
    int minThrottle = 0;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
