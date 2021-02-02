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
#define TAG_PAS_NUM     0x0B
#define TAG_RPM         0x20

#define LEN_START       18
#define LEN_BASIC       24
#define LEN_PEDAL       11
#define LEN_THROTTLE    6

#define DEFAULT_PAS     5

#define WHEEL_RADIUS        0.35f                       // meters
#define WHEEL_CIRCUMFERENCE (2.0f * (float) PI * WHEEL_RADIUS)

#define DAC_MIN_V       0.55f
#define DAC_MAX_V       2.75f
#define REAR_TEETH      20
#define DRIVE_TEETH     44
#define MAX_RPM         120.0f
#define MAX_SPEED       (MAX_RPM * DRIVE_TEETH * WHEEL_CIRCUMFERENCE / REAR_TEETH / 60.0f)

class DriveMotor {
public:
    DriveMotor(int throttle_pin);

    void start();

    bool storeBasic();
    bool storePedal();
    bool storeThrottle();

    void programCurrent(int current, int pas);
    void programSpeed(int speed, int pas);
    void programPAS(int num);

    void setPAS(int num);
    void setSpeed(float speed);

    float getSpeed();

private:
    static byte checksum(long prefactor, int from, int to, const byte *arr);

    float throttleMinV = 1;
    float throttleMaxV = 3;

    int throttlePin = 0;

    byte startBuffer[LEN_START];//array to hold responses from controller
    byte basicBuffer[LEN_BASIC + 3];
    byte pedalBuffer[LEN_PEDAL + 3];
    byte throttleBuffer[LEN_THROTTLE + 3];
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_NEW_DRIVEMOTOR_H
