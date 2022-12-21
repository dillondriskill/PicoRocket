/**
 * @file MPU.c
 * @author Platypushunter
 * @brief Collection of functions related to the MPU
 * @version 0.1
 * @date 2022-12-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MPU.h"

// By default these devices  are on bus address 0x68
static int addr = 0x68;

// Calibration offsets
int16_t AOFF[3] = {-1055, -1078, -1568};
int16_t GOFF[3] = {-712, 851, -994};
// int16_t AOFF[3] = {0, 0, 0};
// int16_t GOFF[3] = {0, 0, 0};

/**
 * @brief Reset the mpu to default settings and calibrate the sensors
 * 
 */
void mpu6050_reset() {

    // Two byte reset. First byte register, second byte data
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);

    // Now to calibrate the sensor and set it to 2000 deg/sec and 16g sensitivity
    // Page 14 of register map has this information
    buf[0] = 0x1B;  // Accel location
    buf[1] = 0b11111000;  // Options
    // Do this 6 times just to really pump it in
    for (int i = 0; i < 6; i ++) {
        i2c_write_blocking(i2c_default, addr, buf, 2, false);
    }

    buf[0] = 0x1C;  // Gyro location
    buf[1] = 0b11111000;  // options

    for (int i = 0; i < 6; i ++) {
        i2c_write_blocking(i2c_default, addr, buf, 2, false);
    }

}

void mpu_calibrate() {
    int8_t AO[6];   // Accel offset in 8bit bytes
    int8_t GO[6];   // Gyro offset in 8bit bytes

    // Turn offsets into 2 8 bit chunks and masking off the 0th bit
    for (int i = 0; i < 3; i++) {
        AOFF[i] &= ~(1UL);
    }
    for (int i = 0; i < 3; i++) {
        AO[i*2] = (AOFF[i] >> 8) & 0xFF;
        AO[i*2+1] = (AOFF[i]) & 0xFF;
        GO[i*2] = (GOFF[i] >> 8) & 0xFF;
        GO[i*2+1] = (GOFF[i]) & 0xFF;
    }
    /*
    // gyro offsets
    uint8_t reg[] = {0x13};
    i2c_write_blocking(i2c_default, addr, reg, 1, true);
    i2c_write_blocking(i2c_default, addr, GO, 6, false);


    // Accel offsets
    reg[0] = 0x06;
    i2c_write_blocking(i2c_default, addr, reg, 1, true);
    i2c_write_blocking(i2c_default, addr, AO, 6, false);
    */
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    // uint8_t val = 0x06; // <-- Offset
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    // val = 0x13; // <-- Offset
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

/**
 * @brief reads in the calibrated data from the mpu and converts it to Gs and deg/sec
 * 
 * @param accel buffer to hold the accelerometer data
 * @param gyro buffer to hold the gyroscope data
 */
void mpu_read(double acc[3], double gy[3]) {
    int16_t acceleration[3], gyro[3];       // Will hold the raw data

    // read raw data
    mpu6050_read_raw(acceleration, gyro);
    
    // Calibrate and convert - Note: we need to have two different variables b/c gy and acc have to be doubles and mpu uses int8_t
    for (int i = 0; i < 3; i++) {
            // calibrate
            acceleration[i] += AOFF[i];
            gyro[i] += GOFF[i];
            // convert
            acc[i] = acceleration[i] / 2048.0;
            gy[i] = gyro[i] / 16.4;
        }

}