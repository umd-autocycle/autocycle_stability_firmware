//
// Created by Misha on 6/24/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_IMU_H
#define AUTOCYCLE_STABILITY_FIRMWARE_IMU_H

#define GRAV            9.8f

#define CALIB_DISCARD   50
#define CALIB_SAMP      200
#define CALIB_A_TOL     16
#define CALIB_G_TOL     4

#define GYRO_RANGE      32767
#define ACCEL_RANGE     32767

#define IMU_TO_ORIGIN_X .62f
#define IMU_TO_ORIGIN_Z .76f

class IMU {
public:
    explicit IMU(uint8_t addr);

    bool start();
    bool configure(uint8_t accel_res, uint8_t gyro_res, uint8_t filtering);
    bool calibrateGyroBias();
    bool calibrateAccelBias(float x_expected, float y_expected, float z_expected);

    // Retrieve corrected accelerometer values in m/s
    float accelX() const;
    float accelY() const;
    float accelZ() const;

    // Retrieve corrected gyroscope values in rad/s
    float gyroX() const;
    float gyroY() const;
    float gyroZ() const;

    // Retrieve chip temperature in degrees Celsius
    float chipTemp() const;

    // Update readings from hardware
    void update();

    void get_gyro_offsets(int16_t & gx_off, int16_t & gy_off, int16_t & gz_off);
    void get_accel_offsets(int16_t & ax_off, int16_t & ay_off, int16_t & az_off);
    void set_gyro_offsets(int16_t gx_off, int16_t gy_off, int16_t gz_off);
    void set_accel_offsets(int16_t ax_off, int16_t ay_off, int16_t az_off);

private:
    void set_register(uint8_t reg, uint8_t val) const;
    void set_register(uint8_t reg, int16_t val) const;
    void read_register(uint8_t reg, uint8_t *val) const;          // Read a single register as byte
    void read_register(uint8_t reg, int16_t *val) const;          // Read two byte register as signed integer
    void read_registers(uint8_t reg, uint8_t val[], int n) const; // Read n single byte registers
    void read_registers(uint8_t reg, int16_t val[], int n) const; // Read n two byte signed int registers

    uint8_t addr;
    int16_t a_x_raw, a_y_raw, a_z_raw;
    int16_t g_x_raw, g_y_raw, g_z_raw;
    int16_t temp_raw;
    float a_x, a_y, a_z;
    float g_x, g_y, g_z;
    float alphaX, alphaZ;
    float last_gyro_x = 0;
    float last_gyro_z = 0;
    float temp;
    float rotation = 16.0 *  PI / 180.0f;

    // Gyroscope and accelerometer FS_SEL and AFS_SEL register resolution values
    uint8_t fs_sel;
    uint8_t afs_sel;

    // Gyroscope and accelerometer full scale ranges
    int gyro_fsr;       // in +/- deg/s
    int accel_fsr;      // in +/- g

    // Tables of gyro and accelerometer FSRs corresponding to FS_SEL and AFS_SEL register values (0-3)
    const int GYRO_FSR[4] = {250, 500, 1000, 2000};     // in +/- g
    const int ACCEL_FSR[4] = {2, 4, 8, 16};             // in +/- deg/s
};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_IMU_H
