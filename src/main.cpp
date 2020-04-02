//
// Created by Cooper Grill on 3/31/2020.
//

#include <Arduino.h>
#include <Wire.h>
//#include <../.pio/libdeps/due/Scheduler_ID884/src/Scheduler.h>

float currentSpeed;
float desiredSpeed;
float currentRoll=0;
float desiredRoll;

//receive input from imu (in progress)
//convert python PID interpolated to C++ (need help from Jack)
//send torque info to motor (need actual motor/specs to complete this)
void maintainStability() {
    delay(10);

    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    double ax, ay, az, gx, gy, gz;

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    raw_ax=Wire.read()<<8|Wire.read();
    raw_ay=Wire.read()<<8|Wire.read();
    raw_az=Wire.read()<<8|Wire.read();
    raw_gx=Wire.read()<<8|Wire.read();
    raw_gy=Wire.read()<<8|Wire.read();
    raw_gz=Wire.read()<<8|Wire.read();

    ax=9.81*raw_ax/16384; ay=9.81*raw_ay/16384; az=9.81*raw_az/16384;  //converts into m/s^2
    gx=(double)raw_gx/131; gy=(double)raw_gy/131; gz=(double)raw_gz/131;    //converts into deg/s

    Serial.print("AX = "); Serial.print(ax);
    Serial.print(" | AY = "); Serial.print(ay);
    Serial.print(" | AZ = "); Serial.print(az);
    Serial.print(" | GX = "); Serial.print(gx);
    Serial.print(" | GY = "); Serial.print(gy);
    Serial.print(" | GZ = "); Serial.println(gz);

    currentRoll=0.98*(currentRoll+gx*0.01)+0.02*atan2(ay, az)*180/PI;    //complementary filter to determine roll
    Serial.print("Roll = "); Serial.println(currentRoll);
}

void maintainSpeed() {

}

void setup() {
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    Serial.begin(9600);
}

void loop() {
    maintainStability();
}