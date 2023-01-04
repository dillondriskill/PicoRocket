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
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MPU.h"
#include "math.h"

#define PI 3.14159265359

// By default these devices  are on bus address 0x68
static int addr = 0x68;


// Calibration offsets - custom to me
int16_t AOFF[3] = {-1050, -1070, -1560};
int16_t GOFF[3] = {-710, 850, -990};

/**
 * @brief Write the data indicated to the mpu
 * 
 * @param buf the data
 * @param len the number of bytes
 */
void mpu_write(uint8_t buf[], int len) {
    i2c_write_blocking(i2c_default, addr, buf, len, false);
}

/**
 * @brief Write the data indicated to the mpu, and keep the line open
 * 
 * @param buf the data
 * @param len the number of bytes
 */
void mpu_write_c(uint8_t buf[], int len) {
    i2c_write_blocking(i2c_default, addr, buf, len, true);
}

/**
 * @brief Reset the mpu to default settings and calibrate the sensors
 * 
 */
void mpu6050_reset() {
    uint8_t buf[] = {0x00, 0x00};

    // Reset the mpu, calibrate the sensors, set the sampling rate and enable and reset the FIFO
    
    // Two byte reset. First byte register, second byte data
    buf[0] = 0x6B;
    buf[0] = 0x00;
    mpu_write(buf, 2);

    // Now to calibrate the sensor and set it to 2000 deg/sec and 16g sensitivity
    // Page 14 of register map has this information
    buf[0] = 0x1B;  // Accel location
    buf[1] = 0b11111000;  // Options
    // Do this 6 times just to really pump it in
    for (int i = 0; i < 6; i ++) {
        mpu_write(buf, 2);
    }

    // test gyro
    buf[0] = 0x1C;  // Gyro location
    buf[1] = 0b11111000;  // options
    for (int i = 0; i < 6; i ++) {
        mpu_write(buf, 2);
    }

    /*
    // Set sampling rate to 8khz / (1 + 7)
    buf[0] = 0x19;  // Sample rate divider register
    buf[1] = 0x07;
    mpu_write(buf, 2);
    */

    /*
    // Turn on the fifo for the accelerometer and the gyro
    buf[0] = 0x23;  // fifo enable reg
    buf[1] = 0b01111000;
    mpu_write(buf, 2);

    // Enable and reset the fifo from user control reg
    buf[0] = 0x6A;  // USER_CTRL
    buf[1] = 0b01000101;    // Enable and reset FIFO
    mpu_write(buf, 2);
    */

}

/**
 * @brief Calibrates the offsets of the mpu. DO NOT USE!!!!!!
 * 
 */
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
    
    // gyro offsets
    uint8_t reg[] = {0x13};
    i2c_write_blocking(i2c_default, addr, reg, 1, true);
    i2c_write_blocking(i2c_default, addr, GO, 6, false);

    // Accel offsets
    reg[0] = 0x06;
    i2c_write_blocking(i2c_default, addr, reg, 1, true);
    i2c_write_blocking(i2c_default, addr, AO, 6, false);
    
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

/**
 * @brief Reads 1 byte from reg
 * 
 * @param reg register to read from 
 * @return uint8_t 
 */
uint8_t mpu_read_reg(uint8_t reg) {
    uint8_t val;

    mpu_write_c(&reg, 1);
    i2c_read_blocking(i2c_default, addr, &val, 1, false);

    return val;
}

/**
 * @brief Reads len number of bytes from reg into buf
 * 
 * @param reg 
 * @param buf 
 * @param len 
 */
void mpu_read_buf(uint8_t reg, uint8_t buf[], int len) {

    mpu_write_c(&reg, 1);
    i2c_read_blocking(i2c_default, addr, buf, len, false);

}

/**
 * @brief Update the position and rotation of the rocket using the MPU FIFO buffer
 * 
 * @param x X position
 * @param y Y position
 * @param z Z position
 * @param pitch Pitch
 * @param yaw Yaw
 * @param roll Roll
 */
void update_postion(double pos[3], double angle[3]) {
    uint8_t reg1 = 0x72;    // FIFO count reg 1
    uint8_t reg2 = 0x73;    // FIFO count reg 2
    uint8_t fifo = 0x74;
    double acc[3], gy[3];
    uint8_t buff[6];
    uint16_t count = ((uint16_t)mpu_read_reg(reg1) << 8) | (uint16_t)mpu_read_reg(reg2);  // Count in FIFO

    
    // Wait until every sensor has had its bytes written
    //if (count != 0) {

        // Lets read each byte and put it where it needs to be
        //while (1) {
            // Read the fifo 6 times for the acc and then turn that into the 16 bit number for each

            for (int i = 0; i < 6; i++) {
                buff[i] = mpu_read_reg(fifo);
            }
            for (int i = 0; i < 3; i++) {
                acc[i] = (((int16_t)buff[i*2] << 8) | ((int16_t)buff[i*2+1])) / 2048.0;     // 2048 is the conversion factor here
            }

            // Read the fifo 6 times for the gy and then turn that into the 16 bit number for each
            for (int i = 0; i < 6; i++) {
                buff[i] = mpu_read_reg(fifo);
            }
            for (int i = 0; i < 3; i++) {
                gy[i] = (((int16_t)buff[i*2] << 8) | ((int16_t)buff[i*2+1])) / 16.4;       // 16.4 is the conversion factor here
            }

            // Update rotational data according to the new data and divide it by the time it took for that reading to happen
            angle[0] += gy[0] / 1000;
            angle[1] += gy[1] / 1000;
            angle[2] += gy[2] / 1000;


            // Now to calculate the position. We need to remove the gravitational component of the accelerometer readings
            acc_cal_grav(acc, angle);

            // Now the acceleration has been calibrated and we can update x y and z
            pos[0] += acc[0] / 1000;
            pos[1] += acc[1] / 1000;
            pos[2] += acc[2] / 1000;
            
            // Update the count
            count = (mpu_read_reg(reg1) << 8) | mpu_read_reg(reg2);

            // Enable and reset the fifo from user control reg
            uint8_t buf[2];
            buf[0] = 0x6A;          // USER_CTRL
            buf[1] = 0b01000101;    // Enable and reset FIFO
            mpu_write(buf, 2);

        //}
    //}
    
}

/**
 * @brief Corrects the accelerometer values for gravity
 * 
 * @param acc the accelerometer data
 * @param angle the angle of the rocket
 */
void acc_cal_grav(double acc[3], double angle[3]) {
    double ang_rad[3];                      // Radian version of the angle
    double grav_ang[3] = {-PI/2, 0, 0};     // The angle of gravity
    double grav_vec[3];                     // The vector of gravity

    // Convert to radians
    ang_rad[0] = (angle[0] * PI / 180);
    ang_rad[1] = (angle[1] * PI / 180);
    ang_rad[2] = (angle[2] * PI / 180);

    // Add our current angle to the gravity angle to find its relative angle
    grav_ang[0] += ang_rad[0];
    grav_ang[1] += ang_rad[1];
    grav_ang[2] += ang_rad[2];

    // Find the relative gravity vector
    grav_vec[0] = cos(ang_rad[1]);
    grav_vec[1] = cos(ang_rad[0]);
    grav_vec[2] = sin(ang_rad[0]);

    // Subtract gravitational components
    acc[0] -= grav_vec[0];
    acc[1] -= grav_vec[1];
    acc[2] -= grav_vec[2];

}