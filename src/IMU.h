//
// Created by Misha on 6/24/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_IMU_H
#define AUTOCYCLE_STABILITY_FIRMWARE_IMU_H

#define GRAV            9.8

#define CALIB_DISCARD   50
#define CALIB_SAMP      200
#define CALIB_A_TOL     16
#define CALIB_G_TOL     4

#define GYRO_RANGE      32767
#define ACCEL_RANGE     32767

class IMU {
public:
    explicit IMU(uint8_t addr);

    bool start();
    bool configure(uint8_t accel_res, uint8_t gyro_res, uint8_t filtering);
    bool calibrateGyros();
    bool calibrateAccel(double x_expected, double y_expected, double z_expected);

    // Retrieve corrected accelerometer values in m/s
    double accelX() const;
    double accelY() const;
    double accelZ() const;

    // Retrieve corrected gyroscope values in rad/s
    double gyroX() const;
    double gyroY() const;
    double gyroZ() const;

    // Retrieve chip temperature in degrees Celsius
    double chipTemp() const;

    // Update readings from hardware
    void update();

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
    double a_x, a_y, a_z;
    double g_x, g_y, g_z;
    double temp;

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
