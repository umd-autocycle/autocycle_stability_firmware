//
// Created by evanr on 9/1/2020.
//

#include "DriveMotor.h"
#include <Arduino.h>

//#define DRIVE_MOTOR_VERBOSE


DriveMotor::DriveMotor(int throttle_pin) {
    throttlePin = throttle_pin;
}

void DriveMotor::start() {
    pinMode(throttlePin, OUTPUT);

    while (DRIVE_SERIAL.available()) DRIVE_SERIAL.read();

    DRIVE_SERIAL.write(TAG_READ);
    DRIVE_SERIAL.write(TAG_START);
    DRIVE_SERIAL.write(0x04);
    DRIVE_SERIAL.write(0xB0);
    DRIVE_SERIAL.write(0x05);


    int c = 0;
    while (c < LEN_START) {
        if (DRIVE_SERIAL.available() > 0) {
            startBuffer[c] = DRIVE_SERIAL.read();
#ifdef DRIVE_MOTOR_VERBOSE
            Serial.print(startBuffer[c]);
            Serial.print(' ');
#endif
            c++;
        }
    }
#ifdef DRIVE_MOTOR_VERBOSE
    Serial.println();
#endif

    storeBasic();
    storePedal();
    storeThrottle();
    setPAS(DEFAULT_PAS);

    throttleMinV = (float) throttleBuffer[3] * 0.1f;
    throttleMaxV = (float) throttleBuffer[4] * 0.1f;

    delay(200);
//    programSpeed();
}


bool DriveMotor::storeBasic() {
    while (DRIVE_SERIAL.available()) DRIVE_SERIAL.read();
    DRIVE_SERIAL.write(TAG_READ);
    DRIVE_SERIAL.write(TAG_BASIC);

    int c = 0;
    while (c < LEN_BASIC + 3) {
        if (DRIVE_SERIAL.available() > 0) {
            basicBuffer[c] = DRIVE_SERIAL.read();
#ifdef DRIVE_MOTOR_VERBOSE
            Serial.print(basicBuffer[c]);
            Serial.print(' ');
#endif
            c++;
        }
    }
#ifdef DRIVE_MOTOR_VERBOSE
    Serial.println();
#endif

    return true;
}


bool DriveMotor::storePedal() {
    while (DRIVE_SERIAL.available()) DRIVE_SERIAL.read();
    DRIVE_SERIAL.write(TAG_READ);
    DRIVE_SERIAL.write(TAG_PEDAL);

    int c = 0;
    while (c < LEN_PEDAL + 3) {
        if (DRIVE_SERIAL.available() > 0) {
            pedalBuffer[c] = DRIVE_SERIAL.read();
#ifdef DRIVE_MOTOR_VERBOSE
            Serial.print(pedalBuffer[c]);
            Serial.print(' ');
#endif
            c++;
        }
    }
#ifdef DRIVE_MOTOR_VERBOSE
    Serial.println();
#endif

    return true;
}

bool DriveMotor::storeThrottle() {
    while (DRIVE_SERIAL.available()) DRIVE_SERIAL.read();
    DRIVE_SERIAL.write(TAG_READ);
    DRIVE_SERIAL.write(TAG_THROTTLE);

    int c = 0;
    while (c < LEN_THROTTLE + 3) {
        if (DRIVE_SERIAL.available() > 0) {
            throttleBuffer[c] = DRIVE_SERIAL.read();
#ifdef DRIVE_MOTOR_VERBOSE
            Serial.print(throttleBuffer[c]);
            Serial.print(' ');
#endif
            c++;
        }
    }
#ifdef DRIVE_MOTOR_VERBOSE
    Serial.println();
#endif

    return true;
}

void DriveMotor::programCurrent(int current, int pas) {
    byte set;
    if (current >= 0 && current <= 100) {
        set = (byte) current;
    } else {
        return;
    }

    basicBuffer[4 + pas] = set;
    basicBuffer[2 + LEN_BASIC] = checksum(0, 0, 2 + LEN_BASIC, basicBuffer);

    DRIVE_SERIAL.write(TAG_WRITE);
    DRIVE_SERIAL.write(TAG_BASIC);
    DRIVE_SERIAL.write(LEN_BASIC);

    for (int i = 2; i < 3 + LEN_BASIC; i++) {
        DRIVE_SERIAL.write(basicBuffer[i]);
    }
    while (!DRIVE_SERIAL.available());
    while (DRIVE_SERIAL.available()) DRIVE_SERIAL.read();
}

void DriveMotor::programSpeed() {

    basicBuffer[14 + 0] = 0;
    basicBuffer[14 + 2] = 13;
    basicBuffer[14 + 4] = 26;
    basicBuffer[14 + 6] = 40;
    basicBuffer[14 + 8] = 53;
    basicBuffer[14 + 9] = 66;
    basicBuffer[2 + LEN_BASIC] = checksum(0, 0, 2 + LEN_BASIC, basicBuffer);

    DRIVE_SERIAL.write(TAG_WRITE);
    DRIVE_SERIAL.write(TAG_BASIC);
    DRIVE_SERIAL.write(LEN_BASIC);
#ifdef DRIVE_MOTOR_VERBOSE
    Serial.print(TAG_WRITE);
    Serial.print(" ");
    Serial.print(TAG_BASIC);
    Serial.print(" ");
    Serial.print(LEN_BASIC);
    Serial.print(" ");
#endif


    for (int i = 2; i < 3 + LEN_BASIC; i++) {
        DRIVE_SERIAL.write(basicBuffer[i]);
#ifdef DRIVE_MOTOR_VERBOSE
        Serial.print(basicBuffer[i]);
        Serial.print(" ");
#endif
    }
    while (!DRIVE_SERIAL.available());
    while (DRIVE_SERIAL.available());
#ifdef DRIVE_MOTOR_VERBOSE
    Serial.println();
#endif
}

void DriveMotor::programPAS(int num) {
    storeThrottle();

    throttleBuffer[5] = num;
    throttleBuffer[2 + LEN_THROTTLE] = checksum(0, 0, 2 + LEN_THROTTLE, throttleBuffer);

    DRIVE_SERIAL.write(TAG_WRITE);
    DRIVE_SERIAL.write(TAG_THROTTLE);
    DRIVE_SERIAL.write(LEN_THROTTLE);

    for (int i = 2; i < 3 + LEN_THROTTLE; i++) {
        DRIVE_SERIAL.write(throttleBuffer[i]);
    }
    delay(1000);
    while (DRIVE_SERIAL.available()) DRIVE_SERIAL.read();

    storePedal();
    pedalBuffer[3] = num;
    pedalBuffer[2 + LEN_PEDAL] = checksum(0, 0, 2 + LEN_PEDAL, pedalBuffer);

    DRIVE_SERIAL.write(TAG_WRITE);
    DRIVE_SERIAL.write(TAG_PEDAL);
    DRIVE_SERIAL.write(LEN_PEDAL);

    for (int i = 2; i < 3 + LEN_PEDAL; i++) {
        DRIVE_SERIAL.write(pedalBuffer[i]);
    }
    while (!DRIVE_SERIAL.available());
    while (DRIVE_SERIAL.available()) DRIVE_SERIAL.read();
}


byte DriveMotor::checksum(long prefactor, int from, int to, const byte *arr) {
    long checksum = prefactor;
    for (int i = from; i < to; i++)
        checksum += arr[i];
    checksum %= 256;

    return checksum;
}

void DriveMotor::setPAS(int num) {
    DRIVE_SERIAL.write(TAG_WRITE);
    DRIVE_SERIAL.write(TAG_PAS_NUM);

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

    DRIVE_SERIAL.write(b1);
    DRIVE_SERIAL.write(b2);
}

void DriveMotor::setSpeed(float speed) {
    float throttleVoltage = (speed / MIN_SPEED) * (throttleMaxV - throttleMinV) + throttleMinV;
    unsigned long throttleLevel = min(4095l * (throttleVoltage - DAC_MIN_V) / (DAC_MAX_V - DAC_MIN_V), 4095l);

    throttleLevel = speed > 0 ? 4095l : 0;
#ifdef DRIVE_MOTOR_VERBOSE
    Serial.println((int) (speed * 10 / MAX_SPEED));
    Serial.println(throttleLevel);
#endif

    setPAS((int) (speed * 10 / MAX_SPEED));

    analogWrite(throttlePin, throttleLevel);
}

float DriveMotor::getSpeed() {
    while (DRIVE_SERIAL.available()) DRIVE_SERIAL.read();
    DRIVE_SERIAL.write(TAG_READ);
    DRIVE_SERIAL.write(TAG_RPM);

    int c = 0;
    while (c < 3) {
        if (DRIVE_SERIAL.available() > 0) {
            startBuffer[c] = DRIVE_SERIAL.read();
            c++;
        }
    }

    int rpm = startBuffer[0] * 256 + startBuffer[1];

    return WHEEL_CIRCUMFERENCE * (float) rpm / 60.0;
}
