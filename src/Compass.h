//
// Created by Misha on 1/23/2022.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_COMPASS_H
#define AUTOCYCLE_STABILITY_FIRMWARE_COMPASS_H

#include <Adafruit_LIS3MDL.h>

#define HARD_IRON_X -46.25
#define HARD_IRON_Y 41.23
#define HARD_IRON_Z -31.99

#define MPS_2_DEGLATPS (1.0f / (1852.0f * 60.0f)) // Meters per second to degrees of latitude per second conversion factor
// A nautical mile is a minute of arc and is 1852.
#define MPS_2_DEGLONPS(lat) (1.0f / (1852.0f * 60.0f * cos((lat) * ((float)PI/180.0f)))) // Meters per second to degrees of longitude per second conversion factor
// Linear distance between degrees of longitude is latitude dependent

class Compass {
public:
    Compass();

    void start();

    void update();

    float angle;
    float rotation;
    float declination, deviation;

private:
    Adafruit_LIS3MDL magnetometer;
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_COMPASS_H
