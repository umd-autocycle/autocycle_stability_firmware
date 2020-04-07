//
// Created by Cooper Grill on 3/31/2020.
//

#include <Arduino.h>
#include <Wire.h>
#include <Scheduler.h>

float currentSpeed = 0; //in m/s
float desiredSpeed; //in m/s
float currentRoll;
float desiredRoll;
int maxThrottle = 7; // in m/s, need to determine what max throttle actually corresponds to
int minThrottle = 0;
const throttlePin = A0; //this is the pin that will control throttle/speed output, to be determined which one for sure

//for speed
float delT1 = 0;
float delT2;
float radius = .35; //dummy value in inches, need to measure
float circumference = 2 * 3.14 * radius;
const interruptPin = 26;

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
    float speedKp = .01;
    float speedKd = .01;
    float speedPreError = 0;
    float dt = .01;
    float outputMax = .5;
    float outputMin = -.5;

    //testing maintainSpeed
    //assume starting units are already in m/s, and matching output units (also in m/s)

    float speedError = desiredSpeed - currentSpeed;

    while(speedError > .01 || speedError < -.01) {
        float pOut = speedKp * speedError;
        float speedDeriv = (speedError - speedPreError) / dt;
        float dOut = speedKd * speedDeriv;
        float output = dOut + pOut;

        if (output > outputMax)
            output = outputMax;
        else if (output < outputMin)
            output = outputMin;

        //currentSpeed += output; // this is just for testing, output will eventually actually send a signal
        speedPreError = speedError;
        speedError = desiredSpeed - currentSpeed;
        int newThrottle = int((currentSpeed += output)/(maxThrottle-minThrottle)*1024);
        analogWrite(throttlePin, newThrottle);
    }
}

void updateSpeed()
{
    delT2 = millis();
    if(delT1 = 0)
    {
        currentSpeed = 0;
    }
    else
    {
        currentSpeed = circumference/(delT2-delT1);
    }
    delT1 = delT2;
}

void setup() {
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    Serial.begin(9600);

    currentRoll=0;

    //speed stuff
    speedCounter = maxSpeedCounter;
    circumference = 2*3.14*radius;
    pinMode(interruptPin, INPUT);
    pinMode(throttlePin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(interruptPin), updateSpeed, RISING);
    //anytime the speed pin goes from low to high, this interrupt should update speed accordingly

    Scheduler.startLoop(maintainStability);
}

void loop() {
    delay(100);
}