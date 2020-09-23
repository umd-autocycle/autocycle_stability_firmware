//
// Created by evanr on 9/1/2020.
//

#include "DriveMotor.h"
#include <Arduino.h>

DriveMotor::DriveMotor(float desiredSpeed) {
    /*this->speedSensorPin = speedSensorPin;
    pinMode(speedSensorPin, INPUT);*/
}

void DriveMotor::start() {  //use [] {drivemotor.convertSignalToSpeed() in main}
    /*attachInterrupt(digitalPinToInterrupt(speedSensorPin), func, RISING);*/
    byte startupArray[5] = {0x11, 0x51, 0x04, 0xB0, 0x05};
    for (int i = 0; i < 5; i++) {
        Serial1.write(startupArray[i]);
    }
    int byteNo = 0;



    //monitor messages from controller until maxwaittime is exceeded
    while (byteNo < 18) {
        if (Serial1.available() > 0) {
            byte cByte = Serial1.read();

            //place received byte in array
            controllerResponse[byteNo] = cByte;

            if (byteNo < 18) {
                byteNo = byteNo + 1;
            }
            Serial.print(cByte, HEX);
            Serial.println();

        }


    }
    storeBasic();
    storePedalAssist();
    storeThrottle();
    startup();
}

void DriveMotor::startup() {

    //now, set pedal assist to 0
    PASResponse[3] = 0x00;
    Serial1.write(0x16);
    Serial1.write(0x53);
    Serial1.write(0x11); //these three commands begin the setting process
    for (int i = 3; i < 14; i++) {
        Serial1.write(PASResponse[i]);
    }
    throttleResponse[5] = 0x00;
    //cannot write to throttle without more info. Check this!
}

bool DriveMotor::storeBasic() {
    byte com1 = 0x11;
    byte com2 = 0x52;

    Serial1.write(com1);
    Serial1.write(com2);
    int byteNo = 0;
    while (byteNo < 26) {
        if (Serial1.available() > 0) {
            byte cByte = Serial1.read();
            //place received byte in array
            basicResponse[byteNo] = cByte;
            byteNo = byteNo + 1;
            Serial.print(cByte, HEX);
            Serial.println();
        }
        /*else
        {
            Serial.print("Failed to store data");
            return false;
        }*/
    }

}

bool DriveMotor::storePedalAssist() {
    byte com1 = 0x11;
    byte com2 = 0x53;

    Serial1.write(com1);
    Serial1.write(com2);
    int byteNo = 0;
    while (byteNo < 13) {
        if (Serial1.available() > 0) {
            byte cByte = Serial1.read();
            //place received byte in array
            PASResponse[byteNo] = cByte;
            byteNo = byteNo + 1;
        }
        else
        {
            return false;
        }
    }
}

bool DriveMotor::storeThrottle() {
    byte com1 = 0x11;
    byte com2 = 0x54;

    Serial1.write(com1);
    Serial1.write(com2);
    int byteNo = 0;
    while (byteNo < 8) {
        if (Serial1.available() > 0) {
            byte cByte = Serial1.read();
            //place received byte in array
            throttleResponse[byteNo] = cByte;
            byteNo = byteNo + 1;
        }
        else
        {
            return false;
        }
    }

}

void DriveMotor::queryRPM() {
    Serial1.write(DSpeed[0]);
    Serial1.write(DSpeed[1]);
    readMotorSignal(true);
}

void DriveMotor::readMotorSignal(bool askingRPM) {
    if (Serial1.available() > 0 && askingRPM == false) {
        Serial2.write(Serial1.read()); //send data along to display
    } else if (Serial1.available() > 0 && askingRPM == true) {


        int byteNo = 0;



        //monitor messages from controller until maxwaittime is exceeded
        while (byteNo < 3) {
            if (Serial1.available() > 0) {
                byte cByte = Serial1.read();

                //place received byte in array
                controllerResponse[byteNo] = cByte;

                if (byteNo < 18) {
                    byteNo = byteNo + 1;
                }


            }


        }
        currentSpeed = convertSignalToSpeed(controllerResponse);
        writeSpeedToMotor(maintainSpeed(desiredSpeed, currentSpeed));
    }
}

void DriveMotor::readDisplaySignal() {
    if (Serial2.available() > 0) {
        int maxWaittime = 200; //If nothing is received after  200 miliseconds, controller finished sending response
        bool exceeded_maxWaittime = false;
        unsigned long waitUntil;
        int byteNo = 0;

        waitUntil = millis() + maxWaittime;

        //monitor messages from controller until maxwaittime is exceeded
        while (!exceeded_maxWaittime) {
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
    if (controllerResponse[0] != 0x11 &&
        controllerResponse[1] != 0x20) { // we can alter this to police any type of messages
        Serial1.write(Serial2.read()); //send data along to display
    }
}


float DriveMotor::convertSignalToSpeed(byte RPMdata[]) {
    float rpm = (RPMdata[1] + RPMdata[0] * 256);
    currentSpeed = (rpm / 60) * 3.14 * radius * 2;
    return currentSpeed; //in m/s
    //populate this function!
    //maintainspeed currently returns a percentage from 0 to 1 of the "max speed" of the bike, since this is the value
    //that we send to the drive motor.
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

void DriveMotor::writeSpeedToMotor(int percentMaxSpeed) {
    basicResponse[4] = percentMaxSpeed; //do i need to change this to hex?
    //also, i'm setting the above as the current percent of max. I think some other calcs may need to be done to confirm this.
    basicResponse[14] = percentMaxSpeed;
    Serial1.write(0x16);
    Serial1.write(0x52);
    Serial1.write(0x24);
    for (int i = 3; i < 27; i++) {
        Serial1.write(basicResponse[i]);
    }
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

    return int((currentSpeed += output) / maxSpeed);
    /*
    return int((currentSpeed += output) / (maxThrottle - minThrottle) * 4096); //what is due analog res?
     */ //What kind of output does the motor need to change speed?
}




