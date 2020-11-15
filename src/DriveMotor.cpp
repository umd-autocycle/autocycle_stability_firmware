//
// Created by evanr on 9/1/2020.
//

#include "DriveMotor.h"
#include <Arduino.h>

DriveMotor::DriveMotor(float desiredSpeed) {

}

void DriveMotor::start() {
    while (Serial1.available()) Serial1.read();

    Serial1.write(TAG_READ);
    Serial1.write(TAG_START);
    Serial1.write(0x04);
    Serial1.write(0xB0);
    Serial1.write(0x05);


    int c = 0;
    while (c < LEN_START) {
        if (Serial1.available() > 0) {
            startBuffer[c] = Serial1.read();
            c++;
        }
    }

    storeBasic();
    storePedal();
    storeThrottle();
}

void DriveMotor::resetMotor() {
    //default settings according to python github
    basicBuffer[0] = 0x52;
    basicBuffer[1] = 0x18;
    basicBuffer[2] = 0x1F;
    basicBuffer[3] = 0x0F;
    basicBuffer[4] = 0x00;
    basicBuffer[5] = 0x1C;
    basicBuffer[6] = 0x25;
    basicBuffer[7] = 0x2E;
    basicBuffer[8] = 0x37;
    basicBuffer[9] = 0x40;
    basicBuffer[10] = 0x49;
    basicBuffer[11] = 0x52;
    basicBuffer[12] = 0x5B;
    basicBuffer[13] = 0x64;
    basicBuffer[14] = 0x64;
    basicBuffer[15] = 0x64;
    basicBuffer[16] = 0x64;
    basicBuffer[17] = 0x64;
    basicBuffer[18] = 0x64;
    basicBuffer[19] = 0x64;
    basicBuffer[20] = 0x64;
    basicBuffer[21] = 0x64;
    basicBuffer[22] = 0x64;
    basicBuffer[23] = 0x64;
    //not setting 24-26 cause we shouldn't be messing with those settings anyways

    pedalBuffer[0] = 0x53;
    pedalBuffer[1] = 0x0B;
    pedalBuffer[2] = 0x03;
    pedalBuffer[3] = 0xFF;
    pedalBuffer[4] = 0xFF;
    //not setting anything past 4 cause we shouldn't be messing with those settings in the first place

    throttleBuffer[0] = 0x54;
    throttleBuffer[1] = 0x06;
    throttleBuffer[3] = 0x00;
    throttleBuffer[4] = 0x03;

    Serial1.write(0x16);
    Serial1.write(0x52);
    Serial1.write(0x24);
    for (int i = 3; i < 27; i++) {
        Serial1.write(basicBuffer[i]);
    }

    Serial1.write(0x16);
    Serial1.write(0x53);
    Serial1.write(0x11);
    for (int i = 3; i < 15; i++) {
        Serial1.write(pedalBuffer[i]);
    }

    Serial1.write(0x16);
    Serial1.write(0x54);
    Serial1.write(0x06);
    for (int i = 3; i < 9; i++) {
        Serial1.write(throttleBuffer[i]);
    }
}


bool DriveMotor::storeBasic() {
    while (Serial1.available()) Serial1.read();
    Serial1.write(TAG_READ);
    Serial1.write(TAG_BASIC);

    int c = 0;
    while (c < LEN_BASIC + 3) {
        if (Serial1.available() > 0) {
            pedalBuffer[c] = Serial1.read();
            c++;
        }
    }

    return true;
}


bool DriveMotor::storePedal() {
    while (Serial1.available()) Serial1.read();
    Serial1.write(TAG_READ);
    Serial1.write(TAG_PEDAL);

    int c = 0;
    while (c < LEN_PEDAL + 3) {
        if (Serial1.available() > 0) {
            pedalBuffer[c] = Serial1.read();
            c++;
        }
    }

    return true;
}

bool DriveMotor::storeThrottle() {
    while (Serial1.available()) Serial1.read();
    Serial1.write(TAG_READ);
    Serial1.write(TAG_THROTTLE);

    int c = 0;
    while (c < LEN_THROTTLE + 3) {
        if (Serial1.available() > 0) {
            throttleBuffer[c] = Serial1.read();
            c++;
        }
    }

    return true;
}

void DriveMotor::programCurrent(int current, int pas) {
    byte set;
    if (current >= 0 && current < 101) {
        set = (byte) current;
    } else {
        Serial.write("Invalid value for max current. Please choose a percentage between 0 and 100.");
        return;
    }

    basicBuffer[4 + pas] = set;
    basicBuffer[2 + LEN_BASIC] = checksum(0, 0, 2 + LEN_BASIC, basicBuffer);

    Serial1.write(TAG_WRITE);
    Serial1.write(TAG_BASIC);
    Serial1.write(LEN_BASIC);

    for (int i = 2; i < 3 + LEN_BASIC; i++) {
        Serial1.write(basicBuffer[i]);
    }

    while (Serial1.available()) Serial1.read();
}

void DriveMotor::programSpeed(int speed, int pas) {
    byte set;
    if (speed >= 0 && speed < 101) {
        set = (byte) speed;
    } else {
        Serial.write("Invalid value for max speed. Please choose a percentage between 0 and 100.");
        return;
    }

    basicBuffer[14 + pas] = set;
    basicBuffer[2 + LEN_BASIC] = checksum(0, 0, 2 + LEN_BASIC, basicBuffer);

    Serial1.write(TAG_WRITE);
    Serial1.write(TAG_BASIC);
    Serial1.write(LEN_BASIC);

    for (int i = 2; i < 3 + LEN_BASIC; i++) {
        Serial1.write(basicBuffer[i]);
    }

    while (Serial1.available()) Serial1.read();
}

void DriveMotor::programPAS(int num) {
    storeThrottle();
    throttleBuffer[5] = num;
    throttleBuffer[2 + LEN_THROTTLE] = checksum(0, 0, 2 + LEN_THROTTLE, throttleBuffer);

    Serial1.write(TAG_WRITE);
    Serial1.write(TAG_THROTTLE);
    Serial1.write(LEN_THROTTLE);

    for (int i = 2; i < 3 + LEN_THROTTLE; i++) {
        Serial1.write(throttleBuffer[i]);
    }
    while (Serial1.available()) Serial1.read();

    storePedal();
    pedalBuffer[3] = num;
    pedalBuffer[2 + LEN_PEDAL] = checksum(0, 0, 2 + LEN_PEDAL, pedalBuffer);

    Serial1.write(TAG_WRITE);
    Serial1.write(TAG_PEDAL);
    Serial1.write(LEN_PEDAL);

    for (int i = 2; i < 3 + LEN_PEDAL; i++) {
        Serial1.write(pedalBuffer[i]);
    }
    while (Serial1.available()) Serial1.read();
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
                startBuffer[byteNo] = cByte;

                if (byteNo < 18) {
                    byteNo = byteNo + 1;
                }


            }


        }
        currentSpeed = convertSignalToSpeed(startBuffer);
//        writeSpeedToMotor(maintainSpeed(desiredSpeed, currentSpeed));
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
                startBuffer[byteNo] = cByte;

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
    if (startBuffer[0] != 0x11 &&
        startBuffer[1] != 0x20) { // we can alter this to police any type of messages
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


byte DriveMotor::checksum(long prefactor, int from, int to, const byte *arr) {
    long checksum = prefactor;
    for (int i = from; i < to; i++)
        checksum += arr[i];
    checksum %= 256;

    return checksum;
}




