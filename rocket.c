/**
 * @file rocket.c
 * @author Platypushunter
 * @brief Pico Powered guided rocket
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

void i2c_start();

int main() {
    // Start up all the systems and calibrate the sensors
    stdio_init_all();
    i2c_start();
    mpu6050_reset();
    mpu_calibrate();

    double acc[3], gy[3];

    while (1) {
        mpu_read(acc, gy);

        printf("%-5.3f %-5.3f %-5.3f    |    %-5.3f %-5.3f %-5.3f\n", acc[0], acc[1], acc[2], gy[0], gy[1], gy[2]);
        //printf("Gyro. X = %3f, Y = %3f, Z = %3f\n", gy[0], gy[1], gy[2]);
        sleep_ms(50);
    }

    return 0;
}

/**
 * @brief Starts up the i2c interface on pins 6 and 7 (gpio 4 and 5) at 400khz
 * 
 */
void i2c_start() {

    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

}