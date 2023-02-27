
/**
 * @file rocket.c
 * @author Dillon Driskill
 * @brief The main file for a pico based guidance system for a rocket, using an mpu-6050 for position and aceleration measurements. This is currently a work in progress in very early stages of development
 * @version 0.1
 * @date 2023-02-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define MPU_ADDRESS 0x68            // Default i2c address of the MPU
#define MPU_PWR_MGMT_ADDRESS 0x6B   // MPU register for power management
#define ACC_CONFIG_ADDRESS 0x1C     // MPU register for the accerlometer configuration
#define GYRO_CONFIG_ADDRESS 0x1B    // MPU register for the gyroscope managment
#define SMPRT_DIV_ADDRESS 0x19      // MPU register for the sample rate divider
#define LP_CONFIG_ADDRESS 0x1A      // MPU register for the FSYNC and low pass filter
#define USER_CTRL_ADRESS 0x6A       // MPU register for the user control options (fifo and i2c)
#define FIFO_EN_ADDRESS 0x23        // MPU register for enabling the fifo
#define ACC_START_ADDRESS 0x3B      // MPU register for the first of the 6 acceleromter registers
#define GYRO_START_ADDRESS 0x43     // MPU register for the first of the 6 gyroscope registers
#define FIFO

// Options for the MPU Registers

#define MPU_PWR_RESET 0x00  // Restarts the MPU

/**
 * Configures the acclerometer to reset itself, and test the X Y and Z axes
 * 
 * Bit 7:
 * X self test | 1 for true 0 for false
 * 
 * Bit 6:
 * Y self test | 1 for true 0 for false
 * 
 * Bit 7:
 * Z self test | 1 for true 0 for false
 * 
 * Bit 4 and 3:
 * Accelerometer scale range
 * 00 - 2g
 * 01 - 4g
 * 10 - 8g
 * 11 - 16g
 *
 * Bit 2, 1, and 0:
 * Reserved
 * 
 * Full reference can be found on page 15:
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf
 */
#define ACC_CONFIG_OPTIONS 0b11111000

/**
 * Configures the gyroscope to reset itself, and test the X Y and Z axes
 * 
 * Bit 7:
 * X self test | 1 for true 0 for false
 * 
 * Bit 6:
 * Y self test | 1 for true 0 for false
 * 
 * Bit 7:
 * Z self test | 1 for true 0 for false
 * 
 * Bit 4 and 3:
 * Accelerometer scale range
 * 00 - 250 deg/sec
 * 01 - 500 deg/sec
 * 10 - 1000 deg/sec
 * 11 - 2000 deg/sec
 *
 * Bit 2, 1, and 0:
 * Reserved
 * 
 * Full reference can be found on page 14:
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf
 */
#define GYRO_CONFIG_OPTIONS 0b11111000

/**
 * Sets the sample rate divider to 1khz.
 * 
 * Sample rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 * 
 * Gyroscope will be set to 8 Khz by default with no lp filter
 * 
 * Full reference can be found on page 11:
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf
 */
#define SMPRT_DIV_OPTIONS 0x07

/**
 * Configures the low pass filter. Filter options can be configured with the following bit mappings:
 * 
 * Bits 7-3:
 * Not used in this case
 * 
 * Bits 2-0:
 * DLP_CFG
 * 
 * DLP_CFG options include:
 * 
 * | DLP_CFG | Acc Bandwidth | Gyro bandwidth | Gryo Khz |
 * |       0 |           260 |            256 |        8 |
 * |       1 |           184 |            188 |        1 |
 * |       2 |            94 |             98 |        1 |
 * |       3 |            44 |             42 |        1 |
 * |       4 |            21 |             20 |        1 |
 * |       5 |            10 |             10 |        1 |
 * |       6 |             5 |              5 |        1 |
 * |       7 | RESERVED      | RESERVED       |        8 |
 * 
 * Full reference can be found on page 11:
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf
 */
#define LP_CONFIG_OPTIONS 0x00

/**
 * Enables the FIFO, resets the FIFO buffer, and resets
 * the signal paths and registers for all sensors
 * 
 * The settings can be configured using the following bit mappings:
 * 
 * Bit 7:
 * Unused
 * 
 * Bit 6:
 * FIFO_EN - enables FIFO
 * 
 * Bit 5:
 * I2C_MST_EN - turns MPU into I2C master mode, instead of slave mode
 * 
 * Bit 4:
 * I2C_IF_DIS - disables the I2C interface
 * 
 * Bit 3:
 * Unused
 * 
 * Bit 2:
 * FIFO_RESET - clears the fifo buffer
 * 
 * Bit 1:
 * I2C_MST_RESET - sets I2C interface to slave mode
 * 
 * Bit 0:
 * SIG_COND_RESET - resets the signal path and registers for all sensors
 * 
 * Full reference can be found on page 38:
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf
 * 
 */
#define USER_CTRL_OPTIONS 0b01000101

/**
 * Enables the FIFO for the gyroscope and acceleromters, disables the temperature and slv sensors
 * 
 * Sensors are enabled according to the following bit mappings:
 * 
 * Bit 7:
 * TEMP_FIFO_EN
 * 
 * Bit 6:
 * XG_FIFO_EN
 * 
 * Bit 5:
 * YG_FIFO_EN
 * 
 * Bit 4:
 * ZG_FIFO_EN
 * 
 * Bit 3:
 * ACELL_FIFO_EN
 * 
 * Bits 2-0
 * Slv2-0 enable
 * 
 * Full reference can be found on page 16:
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf
 */
#define FIFO_EN_OPTIONS 0b01111000

#ifndef M_PI
    #define M_PI 3.14159265358979323846264338327950288
#endif

#define PI M_PI

/**
 * @brief Removes the gravitational component from the accerlomter readingss
 * 
 * @param acc The accelerometer data to be corrected
 * @param angle The current rocket heading as a 3 byte euler angle
 */
void acc_cal_grav(double acc[3], const double angle[3]) {
    /**
     * If you really think about it... This function should be called frame_shift()...
     *
     * To correct the accelerometer readings for gravity we need to know the angle that the rocket is facing.
     * Once we know this angle we can use 3 rotation matrixes (which use a lot of googling to find) to find the correct x y and z components.
     * I could have figured it out with trig and a lot of thought but I dont have time for that
     * 
     * We need to perform the calculation in 3 steps:
     * 1) Rotate around ROLL
     * 2) Rotate around PITCH
     * 3) Rotate around YAW
     * 
     * Look up wiki rotation matrix for more information
     */

    double gravity[3] = {0, 0, -1};         // Vector of gravity on the world plane
    double rad[3] = {angle[0]*PI/180.0,     // Euler angle of the rocket in radians
                     angle[1]*PI/180.0,
                     angle[3]*PI/180.0};

 // gravity[0] we are roating around this axis
    gravity[1] *= cos(rad[0]) - sin(rad[0]);
    gravity[2] *= sin(rad[0]) + cos(rad[0]);

    gravity[0] *= cos(rad[1]) + sin(rad[1]);
 // gravity[1] we are roating around this axis
    gravity[2] *= -sin(rad[1]) + cos(rad[1]);

    gravity[0] *= cos(rad[2]) - sin(rad[2]);
    gravity[1] *= sin(rad[2]) + cos(rad[2]);
 // gravity[2] we are roating around this axis


    // Subtract the calculate gravitational vector from the current acceleration
    acc[0] -= gravity[0];
    acc[1] -= gravity[1];
    acc[2] -= gravity[2];

}

/**
 * @brief Reset the MPU By powercycling, configuring the sensors
 * and fifo, and resetting all the registers and fifo
 * 
 */
static void mpu6050_reset() {
    // Two byte packet: First byte register, second byte data
    uint8_t data[2];
    
    data[0] = MPU_PWR_MGMT_ADDRESS;
    data[1] = MPU_PWR_RESET;

    int trying;

    do {
        trying = i2c_write_blocking(i2c_default, MPU_ADDRESS, data, 2, false);
        printf("Error!\n");
    } while (trying == PICO_ERROR_GENERIC);
    

    // Reset accelerometer and set it to +- 16g sensitivity
    data[0] = ACC_CONFIG_ADDRESS;
    data[1] = ACC_CONFIG_OPTIONS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, data, 2, false);

    // Reset gyroscope and set it to +- 2000 deg/sec sensitivity
    data[0] = GYRO_CONFIG_ADDRESS;
    data[1] = GYRO_CONFIG_OPTIONS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, data, 2, false);

    // Set sample rate to 1khz
    data[0] = SMPRT_DIV_ADDRESS;
    data[1] = SMPRT_DIV_OPTIONS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, data, 2, false);

    // Disable low pass filter
    data[0] = LP_CONFIG_ADDRESS;
    data[1] = LP_CONFIG_OPTIONS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, data, 2, false);

    // Enable the FIFO in general and reset it
    data[0] = USER_CTRL_ADRESS;
    data[1] = USER_CTRL_OPTIONS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, data, 2, false);

    // Enable the FIFO for the accelerometer and the Gyroscope
    data[0] = FIFO_EN_ADDRESS;
    data[1] = FIFO_EN_OPTIONS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, data, 2, false);

    // Clear the FIFO and registers again
    data[0] = USER_CTRL_ADRESS;
    data[1] = USER_CTRL_OPTIONS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, data, 2, false);

}

/**
 * @brief Reads in data from the fifo in two 6 long uint8_t bursts, and converts them to 
 * 3 uint16_t arrays. The first 6 are placed in 
 * 
 * @param accel 
 * @param gyro 
*/
static void mpu6050_read_fifo(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = ACC_START_ADDRESS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);    // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // false b/c we are finished with the bus

    // Reconstruct the 8 bit packets to 16 bits
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    val = GYRO_START_ADDRESS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t reg = ACC_START_ADDRESS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);    // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // false b/c we are finished with the bus

    // Reconstruct the 8 bit packets to 16 bits
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    reg = GYRO_START_ADDRESS;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

void initialize() {
    // This will get the board ready for flight
    stdio_init_all();

    // Turn on light cuz why not
    gpio_pull_up(PICO_DEFAULT_LED_PIN);
    gpio_pull_up(27);

    // This will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Set up the mpu
    mpu6050_reset();
}

int main() {
    initialize();

    int16_t acceleration[3], gyro[3];
    double acc[3] = {0, 0, 0};
    double gy[3] = {0, 0, 0};
    double pos[3] = {0, 0, 0};
    double angle[3] = {0, 90, 90};

    int16_t AOFF[3] = {-1051, -7, -1525};
    int16_t GOFF[3] = {-710, 840, -990};
    uint8_t count_8[2] = {0, 0};
    uint16_t count;

    sleep_ms(500);

    while (1) {

        // See fifo count
        uint8_t buf[1] = {0x72};
        i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 1, true);
        i2c_read_blocking(i2c_default, MPU_ADDRESS, count_8, 1, false);
        count = ((uint16_t)count_8[0] << 8) | (uint16_t)count_8[1];
        
        mpu6050_read_raw(acceleration, gyro);

        acc[0] = (((round(acceleration[0] / 10) * 10) + AOFF[0]) / 2048.0);
        acc[1] = (((round(acceleration[1] / 10) * 10) + AOFF[1]) / 2048.0);
        acc[2] = (((round(acceleration[2] / 10) * 10) + AOFF[2]) / 2048.0);

        gy[0] = ((round(gyro[0] / 10) * 10) + GOFF[0]) / 16.3835;
        gy[1] = ((round(gyro[1] / 10) * 10) + GOFF[1]) / 16.3835;
        gy[2] = ((round(gyro[2] / 10) * 10) + GOFF[2]) / 16.3835;

        // acc_cal_grav(acc, angle);

        angle[0] += gy[0] / 55;
        angle[1] += gy[1] / 55;
        angle[2] += gy[2] / 55;

        pos[0] += (acc[0] * 9.8) / 55;
        pos[1] += (acc[1] * 9.8) / 55;
        pos[2] += (acc[2] * 9.8) / 55;
        
        //printf("Pos. X = %8.3f, Y = %8.3f, Z = %8.3f        ", pos[0], pos[1], pos[2]);
        //printf("Ang. X = %8.3f, Y = %8.3f, Z = %8.3f        ", angle[0], angle[1], angle[2]);
        //printf("Fifo = %d\n", count);
        printf("Acc. X = %8.3f, Y = %8.3f, Z = %8.3f        ", acceleration[0], acceleration[1], acceleration[2]);
        printf("Gyr. X = %8.3f, Y = %8.3f, Z = %8.3f\n", gyro[0], gyro[1], gyro[2]);
        sleep_ms(1);

    }

    return 0;
}
