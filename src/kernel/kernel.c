/**
 * @file kernel.c
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief Contains the code for kernel functions that hide complicated hardware functions away from the programs
 * @version 0.1
 * @date 2023-03-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "kernel.h"
#include "../terminal/terminal.h"

#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"

 // USED ONLY FOR THE SERVOS
float clockDiv = 64;
float wrap = 39062;

void initialize() {

    uint8_t reg[2];
    uint8_t buff[6];

    // Set up all the GPIO
    stdio_init_all();
    gpio_pull_up(PICO_DEFAULT_LED_PIN);

    // Set up the servos
    setServoDeg(SERVO1, -45);
    setServoDeg(SERVO2, -45);
    setServoDeg(SERVO3, -45);
    setServoDeg(SERVO4, -45);

    sleep_ms(500);

    setDeg(SERVO1, 45);
    setDeg(SERVO2, 45);
    setDeg(SERVO3, 45);
    setDeg(SERVO4, 45);

    sleep_ms(500);

    setDeg(SERVO1, 0);
    setDeg(SERVO2, 0);
    setDeg(SERVO3, 0);
    setDeg(SERVO4, 0);

    // Set up i2c
    // This will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Set up the gryo
    reg[0] = GYRO_CONTROL_REG;
    reg[1] = GYRO_CONTROL_BYTE;
    i2c_write_blocking(i2c_default, GYRO_ADDR, &reg, 2, false);
    reg[0] = GYRO_CONFIG_REG;
    reg[1] = GYRO_CONFIG_BYTE;
    i2c_write_blocking(i2c_default, GYRO_ADDR, &reg, 2, false);

    // Set up accel
    reg[0] = ACCEL_CONTROL_REG;
    reg[1] = ACCEL_CONTROL_BYTE;
    i2c_write_blocking(i2c_default, ACCEL_ADDR, &reg, 2, false);
    reg[0] = ACCEL_CONFIG_REG;
    reg[1] = ACCEL_CONFIG_BYTE;
    i2c_write_blocking(i2c_default, ACCEL_ADDR, &reg, 2, false);

    // Set up mag
    reg[0] = MAG_CONTROL_REG;
    reg[1] = MAG_CONTROL_BYTE;
    i2c_write_blocking(i2c_default, MAG_ADDR, &reg, 2, false);

    /**
     * @todo Fuck with the RTC
     *
     */
}

void reset() {
    watchdog_enable(1, 1);
    while (1);
}

void boot_reset() {
    reset_usb_boot(0, 0);
}

void load_program() {
    uint8_t pages;
    uint8_t data[FLASH_PAGE_SIZE];
    uint8_t* temp;

    int ints; // Used for interupt handling

    // Fill our data buffer with 0s
    for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
        data[i] = 0x00;
    }

    // Clear our data regions in flash
    ints = save_and_disable_interrupts();
    flash_range_erase((uint32_t)PROGRAM_LENGTH_OFFSET, 4096 * 16);
    restore_interrupts(ints);

    // Tell the host we're ready
    putchar(0x00);

    // Recieve the number of pages
    pages = getchar();
    data[0] = pages;
    ints = save_and_disable_interrupts();
    flash_range_program(PROGRAM_LENGTH_OFFSET, data, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    // Tell the host what we recieved
    temp = (uint8_t*)(XIP_BASE + PROGRAM_LENGTH_OFFSET);
    putchar(*temp);

    // Recieve data
    for (int page = 0; page < *temp; page++) {
        for (int byte = 0; byte < FLASH_PAGE_SIZE; byte++) {
            data[byte] = getchar();
        }
        // Write data
        ints = save_and_disable_interrupts();
        flash_range_program(PROGRAM_OFFSET + (FLASH_PAGE_SIZE * page), data, FLASH_PAGE_SIZE);
        restore_interrupts(ints);

        // Tell host we're good
        putchar(0x00);
    }

    // Restart
    reset();
}

void read_out() {
    uint8_t* length_ptr = (uint8_t*)XIP_BASE + PROGRAM_LENGTH_OFFSET;
    uint8_t* program_ptr = (uint8_t*)XIP_BASE + PROGRAM_OFFSET;

    // Tell host were about to send the data
    putchar(0x00);

    // Send the data
    for (int i = 0; i < (*length_ptr) * FLASH_PAGE_SIZE; i++) {
        putchar(program_ptr[i]);
    }

    // Tell host we're done
    putchar(0xFF);
}

void setMillis(int servoPin, uint millis) {
    pwm_set_gpio_level(servoPin, (millis / 20000.f) * wrap);
}

void setDeg(int servoPin, float deg) {
    // This ridiculous formula is just turning the supplied degree into the required millisecond pulse to be sent to the servo
    // For example, -45 deg is 1000ms, 0 deg is 1500ms, and 45 deg is 2000ms
    // Just mapping one range to the other
    uint mils = ((ZERO_DEG_MILS + MAX_DEG_MILS) / 2) + (deg * (((MAX_DEG_MILS - ZERO_DEG_MILS) / 2) / SERVORANGE));
    setMillis(servoPin, mils);
}

void setServo(int servoPin, uint startMillis) {
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_config config = pwm_get_default_config();

    /*  Set the pwm clock to be divided 64 times, giving us a frequency of
        1953125 hz, which can be wrapped 39062 times to give us an effective
        pulse width of 20 ms or 50 hz
    */
    pwm_config_set_clkdiv(&config, 64);
    pwm_config_set_wrap(&config, 39062);

    pwm_init(slice_num, &config, true);

    setMillis(servoPin, startMillis);
}

void setServoDeg(int servoPin, float deg) {
    // This ridiculous formula is just turning the supplied degree into the required millisecond pulse to be sent to the servo
    // For example, -45 deg is 1000ms, 0 deg is 1500ms, and 45 deg is 2000ms
    // Just mapping one range to the other
    uint mils = ((ZERO_DEG_MILS + MAX_DEG_MILS) / 2) + (deg * (((MAX_DEG_MILS - ZERO_DEG_MILS) / 2) / SERVORANGE));
    setServo(servoPin, mils);
}

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void host_tools() {
    char in;

    do {
        in = getchar();

        // Tell host we're here
        putchar(in);

        if (in == 0x00) {
            uint8_t reg = 0x28;
            uint8_t buff[1];
            i2c_write_blocking(i2c_default, GYRO_ADDR, &reg, 1, true);
            char worked = i2c_read_blocking(i2c_default, GYRO_ADDR, buff, 1, false);

            for (uint8_t i = 0; i < 1; i++) {
                putchar(buff[i]);
                getchar();
            }

            getchar();

        } else if (in == 0x01) {
            // Host wants to send us servo angles
            int8_t angle;

            angle = getchar();
            setDeg(SERVO1, angle);
            putchar(angle);

            angle = getchar();
            setDeg(SERVO2, angle);
            putchar(angle);

            angle = getchar();
            setDeg(SERVO3, angle);
            putchar(angle);

            angle = getchar();
            setDeg(SERVO4, angle);
            putchar(angle);

        } else if (in == 0x02) {
            load_program();
        } else if (in == 0x03) {
            read_out();
        } else if (in == 0x04) {
            boot_reset();
        } else if (in == 'b') {
            printf("\nI2C Bus Scan\n");
            printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

            for (int addr = 0; addr < (1 << 7); ++addr) {
                if (addr % 16 == 0) {
                    printf("%02x ", addr);
                }

                // Perform a 1-byte dummy read from the probe address. If a slave

                // acknowledges this address, the function returns the number of bytes

                // transferred. If the address byte is ignored, the function returns

                // -1.


                // Skip over any reserved addresses.

                int ret;
                uint8_t rxdata;
                if (reserved_addr(addr))
                    ret = PICO_ERROR_GENERIC;
                else

                    ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

                printf(ret < 0 ? "." : "@");
                printf(addr % 16 == 15 ? "\n" : "  ");
            }
            printf("Done.\n");
        }

    } while (in != 0xFF);

    reset();

}
