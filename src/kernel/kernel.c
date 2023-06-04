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
    stdio_init_all();
    gpio_pull_up(PICO_DEFAULT_LED_PIN);

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

void launch_rocket() {
    // todo
    printf("launching rocket!!!\n");
}

void load_program() {
    uint8_t pages;
    uint8_t data[FLASH_PAGE_SIZE];
    uint8_t* temp;
    for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
        data[i] = 0x00;
    }
    int ints; // Used for interupt handling

    // Clear our data regions in flash
    ints = save_and_disable_interrupts();
    flash_range_erase((uint32_t)PROGRAM_LENGTH_OFFSET, 4096 * 16);
    restore_interrupts(ints);

    // Tell the host we're ready
    putchar(OK);

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
        putchar(OK);
    }

    // Restart
    reset();
}

void read_out() {
    uint8_t* length_ptr = (uint8_t*)XIP_BASE + PROGRAM_LENGTH_OFFSET;
    uint8_t* program_ptr = (uint8_t*)XIP_BASE + PROGRAM_OFFSET;

    putchar('\n');
    for (int i = 0; i < (*length_ptr) * FLASH_PAGE_SIZE; i++) {
        if (i % 16 == 0) {
            putchar('\n');
        }
        printf("%02hhX ", program_ptr[i]);
    }
    putchar('\n');
}

void test_flash() {
    const uint32_t sector = 512 * 1024;
    uint8_t buffer[FLASH_PAGE_SIZE];

    disablecurs();
    cls();

    for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
        buffer[i] = i;
    }

    movcurs(30, 3);
    printf("Buffer filled with:");

    for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
        if (i % 16 == 0) {
            movcurs(16, (i/16) + 5);
        }
        printf("%02hhX ", buffer[i]);
    }

    movcurs(26, 23);
    printf("Press any key to continue...");
    getchar();

    cls();

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(sector, FLASH_SECTOR_SIZE);
    flash_range_program(sector, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    const uint8_t* written_data = (uint8_t*)(XIP_BASE + sector);

    movcurs(25, 3);
    printf("Highlighted bytes are errors:");

    for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
        if (i % 16 == 0) {
            movcurs(16, (i/16) + 5);
        }
        if (written_data[i] == i) {
            printf("%02hhX ", written_data[i]);
        } else {
            printf("\x1b[7m\x1b[5m%02hhX\x1b[0m ", written_data[i]);
        }
        
    }
    
    movcurs(26, 23);
    printf("Press any key to continue...");
    getchar();

    cls();
}

void setMillis(int servoPin, uint millis) {
    pwm_set_gpio_level(servoPin, (millis / 20000.f) * wrap);
}

void setDeg(int servoPin, float deg) {
    uint mils = (deg * DEGTOMIL) + 1000;
    setMillis(servoPin, mils);
}

void setServo(int servoPin, uint startMillis) {
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_config config = pwm_get_default_config();

    uint64_t clockspeed = clock_get_hz(5);
    clockDiv = 64;
    wrap = 39062;

    while (clockspeed / clockDiv / 50 > 65535 && clockDiv < 256) clockDiv += 64;
    wrap = clockspeed / clockDiv / 50;

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);

    setMillis(servoPin, startMillis);
}

void setServoDeg(int servoPin, float deg) {
    uint mils = (deg * DEGTOMIL) + 1000;
    setServo(servoPin, mils);
}

void test_servo() {

    printf("Testing servo 1...\n");
    setServoDeg(6, 0);
    sleep_ms(250);
    setDeg(6, 90);
    sleep_ms(250);
    setDeg(6, 45);
    printf("Press enter to continue\n");
    getchar();

    printf("Testing servo 2...\n");
    setServoDeg(7, 0);
    sleep_ms(250);
    setDeg(7, 90);
    sleep_ms(250);
    setDeg(7, 45);
    printf("Press enter to continue\n");
    getchar();

    printf("Testing servo 3...\n");
    setServoDeg(8, 0);
    sleep_ms(250);
    setDeg(8, 90);
    sleep_ms(250);
    setDeg(8, 45);
    printf("Press enter to continue\n");
    getchar();

    printf("Testing servo 4...\n");
    setServoDeg(9, 0);
    sleep_ms(250);
    setDeg(9, 90);
    sleep_ms(250);
    setDeg(9, 45);
    printf("Press enter to continue\n");
    getchar();

}
