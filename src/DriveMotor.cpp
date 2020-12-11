//
// Created by evanr on 9/1/2020.
//

#include "DriveMotor.h"
#include <Arduino.h>

DriveMotor::DriveMotor(int throttle_pin) {
    throttlePin = throttle_pin;
}

void DriveMotor::start() {
    pinMode(throttlePin, OUTPUT);

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
    setPAS(DEFAULT_PAS);
}

//void DriveMotor::resetMotor() {
//    //default settings according to python github
//    basicBuffer[0] = 0x52;
//    basicBuffer[1] = 0x18;
//    basicBuffer[2] = 0x1F;
//    basicBuffer[3] = 0x0F;
//    basicBuffer[4] = 0x00;
//    basicBuffer[5] = 0x1C;
//    basicBuffer[6] = 0x25;
//    basicBuffer[7] = 0x2E;
//    basicBuffer[8] = 0x37;
//    basicBuffer[9] = 0x40;
//    basicBuffer[10] = 0x49;
//    basicBuffer[11] = 0x52;
//    basicBuffer[12] = 0x5B;
//    basicBuffer[13] = 0x64;
//    basicBuffer[14] = 0x64;
//    basicBuffer[15] = 0x64;
//    basicBuffer[16] = 0x64;
//    basicBuffer[17] = 0x64;
//    basicBuffer[18] = 0x64;
//    basicBuffer[19] = 0x64;
//    basicBuffer[20] = 0x64;
//    basicBuffer[21] = 0x64;
//    basicBuffer[22] = 0x64;
//    basicBuffer[23] = 0x64;
//    //not setting 24-26 cause we shouldn't be messing with those settings anyways
//
//    pedalBuffer[0] = 0x53;
//    pedalBuffer[1] = 0x0B;
//    pedalBuffer[2] = 0x03;
//    pedalBuffer[3] = 0xFF;
//    pedalBuffer[4] = 0xFF;
//    //not setting anything past 4 cause we shouldn't be messing with those settings in the first place
//
//    throttleBuffer[0] = 0x54;
//    throttleBuffer[1] = 0x06;
//    throttleBuffer[3] = 0x00;
//    throttleBuffer[4] = 0x03;
//
//    Serial1.write(0x16);
//    Serial1.write(0x52);
//    Serial1.write(0x24);
//    for (int i = 3; i < 27; i++) {
//        Serial1.write(basicBuffer[i]);
//    }
//
//    Serial1.write(0x16);
//    Serial1.write(0x53);
//    Serial1.write(0x11);
//    for (int i = 3; i < 15; i++) {
//        Serial1.write(pedalBuffer[i]);
//    }
//
//    Serial1.write(0x16);
//    Serial1.write(0x54);
//    Serial1.write(0x06);
//    for (int i = 3; i < 9; i++) {
//        Serial1.write(throttleBuffer[i]);
//    }
//}


bool DriveMotor::storeBasic() {
    while (Serial1.available()) Serial1.read();
    Serial1.write(TAG_READ);
    Serial1.write(TAG_BASIC);

    int c = 0;
    while (c < LEN_BASIC + 3) {
        if (Serial1.available() > 0) {
            basicBuffer[c] = Serial1.read();
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
    Serial.println("Read:");
    while (c < LEN_PEDAL + 3) {
        if (Serial1.available() > 0) {
            pedalBuffer[c] = Serial1.read();
            Serial.print(pedalBuffer[c], HEX);
            Serial.print(" ");
            c++;
        }
    }
    Serial.println();


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
    while (!Serial1.available());
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
    while (!Serial1.available());
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
    delay(1000);
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
    while (!Serial1.available());
    while (Serial1.available()) Serial1.read();
}


byte DriveMotor::checksum(long prefactor, int from, int to, const byte *arr) {
    long checksum = prefactor;
    for (int i = from; i < to; i++)
        checksum += arr[i];
    checksum %= 256;

    return checksum;
}

void DriveMotor::setPAS(int num) {
    Serial1.write(TAG_WRITE);
    Serial1.write(TAG_PAS_NUM);

    byte b1 = 0;
    byte b2 = 0;

    switch (num) {
        case 0:
            b1 = 0x00;
            b2 = 0x21;
            break;
        case 1:
            b1 = 0x0B;
            b2 = 0x2C;
            break;
        case 2:
            b1 = 0x0D;
            b2 = 0x2E;
            break;
        case 3:
            b1 = 0x15;
            b2 = 0x36;
            break;
        case 4:
            b1 = 0x17;
            b2 = 0x38;
            break;
        case 5:
            b1 = 0x03;
            b2 = 0x24;
            break;
        default:
            break;
    }

    Serial1.write(b1);
    Serial1.write(b2);
}

void DriveMotor::setSpeed(float speed) {
    if (speed == 0) {
        analogWrite(throttlePin, 0);
    } else {
        analogWrite(throttlePin, 4095);
    }
    byte speedCode = min(0x28, max(0x0F, speed * 60 * 60 / 1000));

    storeThrottle();
    throttleBuffer[6] = speedCode; // Convert speed from m/s to km/hr
    throttleBuffer[2 + LEN_THROTTLE] = checksum(0, 0, 2 + LEN_THROTTLE, throttleBuffer);

    Serial1.write(TAG_WRITE);
    Serial1.write(TAG_THROTTLE);
    Serial1.write(LEN_THROTTLE);

    for (int i = 2; i < 3 + LEN_THROTTLE; i++) {
        Serial1.write(throttleBuffer[i]);
    }
    while (!Serial1.available());
    while (Serial1.available()) {
        delay(20);
        Serial.println(Serial1.read(), HEX);
    }

    setPAS(0);
    delay(100);
    setPAS(DEFAULT_PAS);
}

float DriveMotor::getSpeed() {
    while (Serial1.available()) Serial1.read();
    Serial1.write(TAG_READ);
    Serial1.write(TAG_RPM);

    int c = 0;
    while (c < 3) {
        if (Serial1.available() > 0) {
            startBuffer[c] = Serial1.read();
            c++;
        }
    }

    int rpm = startBuffer[0] * 256 + startBuffer[1];

    return WHEEL_CIRCUMFERENCE * rpm / 60.0;
}
