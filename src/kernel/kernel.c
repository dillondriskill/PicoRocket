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

 // USED ONLY FOR THE SERVOS
float clockDiv = 64;
float wrap = 39062;

void initialize() {

    // Set up all the GPIO
    stdio_init_all();
    gpio_pull_up(PICO_DEFAULT_LED_PIN);

    // Set up the servos
    setServoDeg(SERVO1, 0);
    setServoDeg(SERVO2, 0);
    setServoDeg(SERVO3, 0);
    setServoDeg(SERVO4, 0);

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

void host_tools() {
    char in;

    putchar(0x00);

    do {
        in = getchar();

        if (in == 0x00) {
            // Host wants sensor information

            // TODO: SEND SENSOR INFORMAITON TO HOST
        } else if (in == 0x01) {
            // Host wants to send us servo angles
            int8_t angle;
            
            angle = getchar();
            setDeg(SERVO1, angle);
            putchar(0x00);

            angle = getchar();
            setDeg(SERVO2, angle);
            putchar(0x00);

            angle = getchar();
            setDeg(SERVO3, angle);
            putchar(0x00);

            angle = getchar();
            setDeg(SERVO4, angle);
            putchar(0x00);
        } else if (in == 0x02) {
            // Upload firmware
        }

    } while (in != 0xFF);

}
