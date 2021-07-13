//
// Created by Misha on 6/24/2020.
//

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


#include "IMU.h"

IMU::IMU(int cs_pin) {
    this->cs_pin = cs_pin;

    this->a_x = 0;
    this->a_y = 0;
    this->a_z = 0;
    this->g_x = 0;
    this->g_y = 0;
    this->g_z = 0;

    this->a_x_raw = 0;
    this->a_y_raw = 0;
    this->a_z_raw = 0;
    this->g_x_raw = 0;
    this->g_y_raw = 0;
    this->g_z_raw = 0;

    this->temp = 0;
    this->temp_raw = 0;

    this->gyro_fsr = 0;
    this->accel_fsr = 0;

    this->fs_sel = 0;
    this->afs_sel = 0;
}

bool IMU::start() {
    pinMode(cs_pin, OUTPUT);
    set_register(0x6B, (uint8_t) 0x00); // Set reset and wait
    delay(100);

    digitalWrite(cs_pin, HIGH);
    return true;
}

bool IMU::configure(uint8_t accel_res, uint8_t gyro_res, uint8_t filtering) {
    this->fs_sel = gyro_res;                // Record FS_SEL and AFS_SEL values, look up FSR for each from table
    this->afs_sel = accel_res;
    this->gyro_fsr = GYRO_FSR[gyro_res];
    this->accel_fsr = ACCEL_FSR[accel_res];

    set_register(0x1A, filtering);      // Set DLPF_CFG (low pass filtering) register

    // Set FS_SEL and AFS_SEL (gyro & accel resolution) registers
    set_register(0x1B, (uint8_t) (fs_sel << 3U));
    set_register(0x1C, (uint8_t) (afs_sel << 3U));

    return true;
}

bool IMU::calibrateGyroBias() {
    set_gyro_offsets(0, 0, 0);

    long x_acc = 0;
    long y_acc = 0;
    long z_acc = 0;

    for (int i = 0; i < CALIB_DISCARD; i++) {
        update();
        delay(2);
    }
    for (int i = 0; i < CALIB_SAMP; i++) {
        update();
        x_acc += g_x_raw;
        y_acc += g_y_raw;
        z_acc += g_z_raw;
        delay(2);
    }

    int16_t x_g_offset, y_g_offset, z_g_offset;
    get_gyro_offsets(x_g_offset, y_g_offset, z_g_offset);
    // Serial.println(x_g_offset);
    // Serial.println(y_g_offset);
    // Serial.println(z_g_offset);

    // Serial.println(x_acc / CALIB_SAMP);
    // Serial.println(y_acc / CALIB_SAMP);
    // Serial.println(z_acc / CALIB_SAMP);


    x_g_offset -= x_acc / CALIB_SAMP;
    y_g_offset -= y_acc / CALIB_SAMP;
    z_g_offset -= z_acc / CALIB_SAMP;
    // Serial.println(x_g_offset);
    // Serial.println(y_g_offset);
    // Serial.println(z_g_offset);

    set_gyro_offsets(x_g_offset, y_g_offset, z_g_offset);

    x_acc = 0;
    y_acc = 0;
    z_acc = 0;

    for (int i = 0; i < CALIB_DISCARD; i++) {
        update();
        delay(2);
    }
    for (int i = 0; i < CALIB_SAMP; i++) {
        update();
        x_acc += g_x_raw;
        y_acc += g_y_raw;
        z_acc += g_z_raw;
        delay(2);
    }

    get_gyro_offsets(x_g_offset, y_g_offset, z_g_offset);
    // Serial.println(x_g_offset);
    // Serial.println(y_g_offset);
    // Serial.println(z_g_offset);

    return abs(x_acc / CALIB_SAMP) < CALIB_G_TOL &&
           abs(y_acc / CALIB_SAMP) < CALIB_G_TOL &&
           abs(z_acc / CALIB_SAMP) < CALIB_G_TOL;
}

bool IMU::calibrateAccelBias(float x_expected, float y_expected, float z_expected) {
//    set_accel_offsets(0, 0, 0);
    delay(100);

    long x_acc = 0;
    long y_acc = 0;
    long z_acc = 0;

    for (int i = 0; i < CALIB_DISCARD; i++) {
        update();
        delay(2);
    }
    for (int i = 0; i < CALIB_SAMP; i++) {
        update();
        x_acc += a_x_raw;
        y_acc += a_y_raw;
        z_acc += a_z_raw;
        delay(2);
    }

    int16_t x_a_expected = (x_expected / GRAV / (float) accel_fsr) * ACCEL_RANGE;
    int16_t y_a_expected = (y_expected / GRAV / (float) accel_fsr) * ACCEL_RANGE;
    int16_t z_a_expected = (z_expected / GRAV / (float) accel_fsr) * ACCEL_RANGE;

    int16_t x_a_offset, y_a_offset, z_a_offset;
    get_accel_offsets(x_a_offset, y_a_offset, z_a_offset);
    // Serial.println(x_a_offset);
    // Serial.println(y_a_offset);
    // Serial.println(z_a_offset);

    // Serial.println(x_acc / CALIB_SAMP);
    // Serial.println(y_acc / CALIB_SAMP);
    // Serial.println(z_acc / CALIB_SAMP);

    x_a_offset -= (x_acc / CALIB_SAMP - x_a_expected) / 2;
    y_a_offset -= (y_acc / CALIB_SAMP - y_a_expected) / 2;
    z_a_offset -= (z_acc / CALIB_SAMP - z_a_expected) / 2;
    // Serial.println(x_a_offset);
    // Serial.println(y_a_offset);
    // Serial.println(z_a_offset);

    set_accel_offsets(x_a_offset, y_a_offset, z_a_offset);


    x_acc = 0;
    y_acc = 0;
    z_acc = 0;

    for (int i = 0; i < CALIB_DISCARD; i++) {
        update();
        delay(2);
    }
    for (int i = 0; i < CALIB_SAMP; i++) {
        update();
        x_acc += a_x_raw;
        y_acc += a_y_raw;
        z_acc += a_z_raw;
        delay(2);
    }

    get_accel_offsets(x_a_offset, y_a_offset, z_a_offset);
    // Serial.println(x_a_offset);
    // Serial.println(y_a_offset);
    // Serial.println(z_a_offset);

    return abs(x_acc / CALIB_SAMP - x_a_expected) < CALIB_A_TOL &&
           abs(y_acc / CALIB_SAMP - y_a_expected) < CALIB_A_TOL &&
           abs(z_acc / CALIB_SAMP - z_a_expected) < CALIB_A_TOL;
}

float IMU::accelX() const {
    return a_x * cos(rotation);// - a_z * sin(rotation) + (gyroZ() * gyroZ()) * IMU_TO_ORIGIN_X;
}

float IMU::accelY() const {
    return a_y ;//+ alphaX * IMU_TO_ORIGIN_Z - alphaZ * IMU_TO_ORIGIN_X;
}

float IMU::accelZ() const {
    return a_z ;//* cos(rotation) + a_x * sin(rotation) + (gyroX() * gyroX()) * IMU_TO_ORIGIN_Z;
}

float IMU::gyroX() const {
    return g_x * cos(rotation) - g_z * sin(rotation);
}

float IMU::gyroY() const {
    return g_y;
}

float IMU::gyroZ() const {
    return g_z * cos(rotation) + g_x * sin(rotation);
}

float IMU::chipTemp() const {
    return temp;
}


void IMU::update() {
    static unsigned long last_time = 0;
    float dt = (float) (millis() - last_time) / 1000.0f;
    last_time = millis();

    int16_t buffer[7];
    read_registers(0x3B, buffer, 7);

    a_x_raw = buffer[0];
    a_y_raw = buffer[1];
    a_z_raw = buffer[2];
    temp_raw = buffer[3];
    g_x_raw = buffer[4];
    g_y_raw = buffer[5];
    g_z_raw = buffer[6];

    a_x = GRAV * (float) (a_x_raw * accel_fsr) / ACCEL_RANGE;
    a_y = GRAV * (float) (a_y_raw * accel_fsr) / ACCEL_RANGE;
    a_z = GRAV * (float) (a_z_raw * accel_fsr) / ACCEL_RANGE;

    temp = (float) temp_raw / 340.0f + 36.53f;
    g_x = (float) (g_x_raw * gyro_fsr) / GYRO_RANGE * (float) PI / 180.0f;
    g_y = (float) (g_y_raw * gyro_fsr) / GYRO_RANGE * (float) PI / 180.0f;
    g_z = (float) (g_z_raw * gyro_fsr) / GYRO_RANGE * (float) PI / 180.0f;

    alphaX = ((gyroX() - last_gyro_x)) / dt;
    alphaZ = ((gyroZ() - last_gyro_z)) / dt;

    last_gyro_x = gyroX();
    last_gyro_z = gyroZ();

}

void IMU::set_register(uint8_t reg, uint8_t val) const {
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);
    SPI.transfer(val);
    digitalWrite(cs_pin, HIGH);
}

void IMU::set_register(uint8_t reg, int16_t val) const {
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);
    SPI.transfer((uint8_t) (val >> 8));
    SPI.transfer(val & 0xFF);
    digitalWrite(cs_pin, HIGH);
}

void IMU::read_register(uint8_t reg, uint8_t *val) const {
    reg = reg | 0b10000000;
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);
    *val = SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
}

void IMU::read_register(uint8_t reg, int16_t *val) const {
    reg = reg | 0b10000000;

    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);
    uint8_t v1 = SPI.transfer(0x00);
    uint8_t v2 = SPI.transfer(0x00);
    *val = (int16_t) ((v1 << 8U) | v2);
    digitalWrite(cs_pin, HIGH);
}

void IMU::read_registers(uint8_t reg, uint8_t *val, int n) const {
    reg = reg | 0b10000000;

    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);

    for (int i = 0; i < n; i++)
        val[i] = SPI.transfer(0x00);

    digitalWrite(cs_pin, HIGH);

}

void IMU::read_registers(uint8_t reg, int16_t *val, int n) const {
    reg = reg | 0b10000000;

    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);

    for (int i = 0; i < n; i++){
        uint8_t v1 = SPI.transfer(0x00);
        uint8_t v2 = SPI.transfer(0x00);
        val[i] = (int16_t) ((v1 << 8U) | v2);
    }

    digitalWrite(cs_pin, HIGH);
}

void IMU::set_gyro_offsets(int16_t gx_off, int16_t gy_off, int16_t gz_off) {
    set_register(0x13, gx_off);     // set XG_OFFS (undocumented X gyro offset, uses FS_SEL units)
    set_register(0x15, gy_off);     // set YG_OFFS (undocumented Y gyro offset, uses FS_SEL units)
    set_register(0x17, gz_off);     // set ZG_OFFS (undocumented Z gyro offset, uses FS_SEL units)
}

void IMU::set_accel_offsets(int16_t ax_off, int16_t ay_off, int16_t az_off) {
    set_register(0x06, ax_off); // set XA_OFFS (undocumented X accelerometer offset, uses 1/2 AFS_SEL units)
    set_register(0x08, ay_off); // set YA_OFFS (undocumented Y accelerometer offset, uses 1/2 AFS_SEL units)
    set_register(0x0A, az_off); // set ZA_OFFS (undocumented Z accelerometer offset, uses 1/2 AFS_SEL units)
}

void IMU::get_gyro_offsets(int16_t &gx_off, int16_t &gy_off, int16_t &gz_off) {
    read_register(0x13, (int16_t *) &gx_off);
    read_register(0x15, (int16_t *) &gy_off);
    read_register(0x17, (int16_t *) &gz_off);
}

void IMU::get_accel_offsets(int16_t &ax_off, int16_t &ay_off, int16_t &az_off) {
    read_register(0x06, (int16_t *) &ax_off);
    read_register(0x08, (int16_t *) &ay_off);
    read_register(0x0A, (int16_t *) &az_off);
}

