//
// Created by evanr on 9/1/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
#define AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
#include <Arduino.h>

// Communication constants
#define TAG_READ        0x11
#define TAG_WRITE       0x16

#define TAG_START       0x51
#define TAG_BASIC       0x52
#define TAG_PEDAL       0x53
#define TAG_THROTTLE    0x54

#define LEN_START       18
#define LEN_BASIC       24
#define LEN_PEDAL       11
#define LEN_THROTTLE    6

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

    float convertSignalToSpeed(byte RPMdata[]);
    void readDisplaySignal();

    bool storeBasic();
    bool storePedal();
    bool storeThrottle();

    void programCurrent(int current, int pas);
    void programSpeed(int speed, int pas);
    void programPAS(int num);


private:
    static byte checksum(long prefactor, int from, int to, const byte *arr);

    float delT1 = 0;
    float delT2;
    const float radius = .35; //dummy value in inches, need to measure //const or preprocessor macros
    const float circumference = 2.0f * 3.14f * radius;//const or preprocessor macros
    float currentSpeed = 0;
    float desiredSpeed;
    int maxThrottle = 7;    //in m/s, need to determine what max throttle actually corresponds to
    int minThrottle = 0;

    byte startBuffer[LEN_START];//array to hold responses from controller
    byte basicBuffer[LEN_BASIC + 3];
    byte pedalBuffer[LEN_PEDAL + 3];
    byte throttleBuffer[LEN_THROTTLE + 3];


    const byte DSpeed[2] = {0x11, 0x20}; //command that display sends to get rpm data
    const float maxSpeed=11.1; //40 km/h in m/s, may change
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
