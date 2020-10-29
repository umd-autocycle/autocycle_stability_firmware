//
// Created by evanr on 9/1/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
#define AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
#include <Arduino.h>
//goals of class:
//handle communication btwn due and drive motor
//control set speed
//receive input of digital beeps, convert to speed
//send analog output back to motor

class DriveMotor {
public:
    DriveMotor(float desiredSpeed);

    void start();

    void readMotorSignal(bool askRPM);

    void queryRPM();
    void resetMotor();
    void writeSpeedToMotor(int percentMaxSpeed);
    float convertSignalToSpeed(byte RPMdata[]);
    void readDisplaySignal();
    bool storePedalAssist();
    bool storeBasic();
    bool storeThrottle();
    void startup();

    //void writeAnalog();

    int maintainSpeed(float desiredSpeed, float currentSpeed);


private:

    float delT1 = 0;
    float delT2;
    const float radius = .35; //dummy value in inches, need to measure //const or preprocessor macros
    const float circumference = 2.0f * 3.14f * radius;//const or preprocessor macros
    float currentSpeed = 0;
    float desiredSpeed;
    const int Serial1TX = 18;
    const int Serial1RX = 19;
    const int Serial2TX = 16;
    const int Serial2RX = 17;
    int maxThrottle = 7;    //in m/s, need to determine what max throttle actually corresponds to
    int minThrottle = 0;
    byte controllerResponse[20];//array to hold responses from controller
    byte basicResponse[26];
    byte PASResponse[13];
    byte throttleResponse[8];
    const byte DSpeed[2] = {0x11, 0x20}; //command that display sends to get rpm data
    const float maxSpeed=11.1; //40 km/h in m/s, may change
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
