//
// Created by evanr on 9/1/2020.
//

#include "DriveMotor.h"
#include <Arduino.h>

DriveMotor::DriveMotor(float desiredSpeed) {
    /*this->speedSensorPin = speedSensorPin;
    pinMode(speedSensorPin, INPUT);*/
}

void DriveMotor::start(void (*func)(void)) {  //use [] {drivemotor.convertSignalToSpeed() in main}
    /*attachInterrupt(digitalPinToInterrupt(speedSensorPin), func, RISING);*/
}

void DriveMotor::queryRPM(){
    const byte DSpeed[] = {0x11, 0x20}; //command that display sends to get rpm data
    Serial1.write(DSpeed);
    readMotorSignal(true);
}

void DriveMotor::readMotorSignal(bool askingRPM) {
    if (Serial1.available() > 0 && bool askingRPM == false)
    {
        Serial2.write(Serial1.read()); //send data along to display
    }
    else if (Serial1.available() > 0 && bool askingRPM == true)
    {
        int maxWaittime = 200; //If nothing is received after  200 miliseconds, controller finished sending response
        bool exceeded_maxWaittime = false;
        unsigned long waitUntil;
        int byteNo = 0;

        waitUntil = millis() + maxWaittime;

        //monitor messages from controller until maxwaittime is exceeded
        while (!exceeded_maxWaittime)  {
            if (Serial1.available() > 0) {
                byte cByte = Serial1.read();

                //place received byte in array
                controllerResponse[byteNo] = cByte;

                if (byteNo < 18) {
                    byteNo = byteNo + 1;
                }

                waitUntil = millis() + maxWaittime; //Adjust waitUntill
            }

            if (millis() > waitUntil) {
                exceeded_maxWaittime = true;
            }
        }
        convertSignaltoSpeed(controllerResponse);
    }
}

void readDisplaySignal()
{
    if (Serial2.available() > 0)
    {
        int maxWaittime = 200; //If nothing is received after  200 miliseconds, controller finished sending response
        bool exceeded_maxWaittime = false;
        unsigned long waitUntil;
        int byteNo = 0;

        waitUntil = millis() + maxWaittime;

        //monitor messages from controller until maxwaittime is exceeded
        while (!exceeded_maxWaittime)  {
            if (Serial2.available() > 0) {
                byte cByte = Serial2.read();

                //place received byte in array
                controllerResponse[byteNo] = cByte;

                if (byteNo < 18) {
                    byteNo = byteNo + 1;
                }

                waitUntil = millis() + maxWaittime; //Adjust waitUntill
            }

            if (millis() > waitUntil) {
                exceeded_maxWaittime = true;
            }
        }
    }
        if(controllerResponse[0] != 0x11 && controllerResponse[1] != 0x20) {
            Serial1.write(Serial2.read()); //send data along to display
        }
    }
}

void DriveMotor::convertSignalToSpeed(byte RPMdata[]) {
    float rpm = (RPMdata[1] + RPMdata[0] * 256);
    currentSpeed = (rpm/60)*3.14*radius*2;
    writeSpeedtoMotor(maintainSpeed(desiredSpeed,currentSpeed));//populate this function!
    /*
    delT2 = millis();
    if (delT1 == 0) {
        currentSpeed = 0;
    } else if ((delT2 - delT1) < 100) {
        return;
    } else {
        currentSpeed = circumference / ((delT2 - delT1) / 1000);
    }
    delT1 = delT2;
    writeAnalog();
     */
}

void DriveMotor::writeSpeedToMotor(int speed)
{
    //Here we'll determine the bytes we need to send to the motor to change to desired speed, and send that data.
}

/*
void DriveMotor::writeAnalog() {
    analogWrite(driveMotorPin, maintainSpeed(desiredSpeed, currentSpeed));
}
*/
int DriveMotor::maintainSpeed(float desiredSpeed, float currentSpeed) {
    float speedKp = .01;
    float speedKd = .01;
    float speedPreError = 0;
    float dt = .01;
    float outputMax = .5;
    float outputMin = -.5;

    //testing maintainSpeed
    //assume starting units are already in m/s, and matching output units (also in m/s)

    float speedError = desiredSpeed - currentSpeed;


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
    /*
    return int((currentSpeed += output) / (maxThrottle - minThrottle) * 4096); //what is due analog res?
     */ //What kind of output does the motor need to change speed?
}




