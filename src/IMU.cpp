//
// Created by Misha on 6/24/2020.
//

#include <Arduino.h>
#include <Wire.h>


#include "IMU.h"

IMU::IMU(uint8_t addr) {
    this->addr = addr;

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
    set_register(0x6B, (uint8_t) 0x00); // Set reset and wait
    delay(100);

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

bool IMU::calibrateGyros() {
    int x_acc = 0;
    int y_acc = 0;
    int z_acc = 0;

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

    int16_t x_g_offset;
    int16_t y_g_offset;
    int16_t z_g_offset;

    read_register(0x13, (int16_t *) &x_g_offset);
    read_register(0x15, (int16_t *) &y_g_offset);
    read_register(0x17, (int16_t *) &z_g_offset);

    x_g_offset -= x_acc / CALIB_SAMP;
    y_g_offset -= y_acc / CALIB_SAMP;
    z_g_offset -= z_acc / CALIB_SAMP;

    set_register(0x13, x_g_offset);     // set XG_OFFS (undocumented X gyro offset, uses FS_SEL units)
    set_register(0x15, y_g_offset);     // set YG_OFFS (undocumented Y gyro offset, uses FS_SEL units)
    set_register(0x17, z_g_offset);     // set ZG_OFFS (undocumented Z gyro offset, uses FS_SEL units)

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

    return abs(x_acc / CALIB_SAMP) < CALIB_G_TOL &&
           abs(y_acc / CALIB_SAMP) < CALIB_G_TOL &&
           abs(z_acc / CALIB_SAMP) < CALIB_G_TOL;
}

bool IMU::calibrateAccel(float x_expected, float y_expected, float z_expected) {
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

    int16_t x_a_expected = x_expected / GRAV / (float) accel_fsr * ACCEL_RANGE;
    int16_t y_a_expected = y_expected / GRAV / (float) accel_fsr * ACCEL_RANGE;
    int16_t z_a_expected = z_expected / GRAV / (float) accel_fsr * ACCEL_RANGE;

    int16_t x_a_offset;
    int16_t y_a_offset;
    int16_t z_a_offset;

    read_register(0x06, (int16_t *) &x_a_offset);
    read_register(0x08, (int16_t *) &y_a_offset);
    read_register(0x0A, (int16_t *) &z_a_offset);

    x_a_offset -= (x_acc / CALIB_SAMP - x_a_expected) / 2;
    y_a_offset -= (y_acc / CALIB_SAMP - y_a_expected) / 2;
    z_a_offset -= (z_acc / CALIB_SAMP - z_a_expected) / 2;

    set_register(0x06, x_a_offset); // set XA_OFFS (undocumented X accelerometer offset, uses 1/2 AFS_SEL units)
    set_register(0x08, y_a_offset); // set YA_OFFS (undocumented Y accelerometer offset, uses 1/2 AFS_SEL units)
    set_register(0x0A, z_a_offset); // set ZA_OFFS (undocumented Z accelerometer offset, uses 1/2 AFS_SEL units)

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

    return abs(x_acc / CALIB_SAMP) < CALIB_A_TOL &&
           abs(y_acc / CALIB_SAMP) < CALIB_A_TOL &&
           abs(z_acc / CALIB_SAMP) < CALIB_A_TOL;
}

float IMU::accelX() const {
    return a_x;
}

float IMU::accelY() const {
    return a_y;
}

float IMU::accelZ() const {
    return a_z;
}

float IMU::gyroX() const {
    return g_x;
}

float IMU::gyroY() const {
    return g_y;
}

float IMU::gyroZ() const {
    return g_z;
}

float IMU::chipTemp() const {
    return temp;
}


void IMU::update() {
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

    temp = (float) temp_raw / 340.0 + 36.53f;
    g_x = (float) (g_x_raw * gyro_fsr) / GYRO_RANGE *  PI / 180.0;
    g_y = (float) (g_y_raw * gyro_fsr) / GYRO_RANGE * PI / 180.0;
    g_z = (float) (g_z_raw * gyro_fsr) / GYRO_RANGE *  PI / 180.0;

}

void IMU::set_register(uint8_t reg, uint8_t val) const {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void IMU::set_register(uint8_t reg, int16_t val) const {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write((uint8_t) (val >> 8));
    Wire.write(val & 0xFF);
    Wire.endTransmission();
}

void IMU::read_register(uint8_t reg, uint8_t *val) const {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int) addr, 1, true);
    *val = Wire.read();
}

void IMU::read_register(uint8_t reg, int16_t *val) const {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int) addr, 2, true);
    *val = Wire.read() << 8U | Wire.read();
}

void IMU::read_registers(uint8_t reg, uint8_t *val, int n) const {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int) addr, n, true);

    for (int i = 0; i < n; i++)
        val[i] = Wire.read();
}

void IMU::read_registers(uint8_t reg, int16_t *val, int n) const {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int) addr, 2 * n, true);

    for (int i = 0; i < n; i++)
        val[i] = Wire.read() << 8 | Wire.read();
}

