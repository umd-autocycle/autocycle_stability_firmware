//
// Created by Misha on 1/23/2022.
//

#include "Compass.h"

Compass::Compass() {
    angle = rotation = 0;
    declination = (10.81666666 / 180.0 * PI);
    deviation = -0.08;
}

void Compass::start() {
    if (!magnetometer.begin_I2C()) {
        Serial.println("Couldn't connect to magnetometer.");
        while (true);
    }
    magnetometer.setPerformanceMode(LIS3MDL_HIGHMODE);
    magnetometer.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    magnetometer.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    magnetometer.setRange(LIS3MDL_RANGE_4_GAUSS);
    magnetometer.setIntThreshold(500);
    magnetometer.configInterrupt(false, false, true, // enable z axis
                                 true, // polarity
                                 false, // don't latch
                                 true);
}

void Compass::update() {
    magnetometer.read();
    float xgo = magnetometer.y_gauss * 100.0f - HARD_IRON_Y;
    float ygo = magnetometer.x_gauss * 100.0f - HARD_IRON_X;
    float zgo = magnetometer.z_gauss * 100.0f - HARD_IRON_Z;

    float xgor = xgo * cos(rotation) - zgo * sin(rotation);
//    Serial.print(xgo);
//    Serial.print('\t');
//    Serial.print(ygo);
//    Serial.print('\t');
//    Serial.print(zgo);
//    Serial.print('\t');
//    Serial.print(xgor);
//    Serial.print('\t');
//    Serial.println();

    angle = atan2(-ygo, xgor) + declination + deviation;
}
