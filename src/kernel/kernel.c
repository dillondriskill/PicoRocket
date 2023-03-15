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
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "hardware/flash.h"
#include "kernel.h"


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
    char ack; // acknowledgement from the host
    long int length; // 32 bit integer to hold the length of the program
    uint8_t buff[FLASH_PAGE_SIZE]; // one flash page sized buffer to hold data for the programs as they come in
    // set buffer to all 0s
    for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
        buff[i] = 0x00;
    }

    // Erase the last sector of flash to make sure any old data is gone
    flash_range_erase(LAST_FLASH_PAGE, FLASH_SECTOR_SIZE);

    // Wait for the user/host to send an l to tell us that we are about to recieve the program
    do {
        ack = getchar();
    } while (ack != 'l');

    //now recieve the length of the program as a 32 bit int
    scanf("%ld", &length);

    // load the 32 bit into the page buffer
    buff[0] = length & 0xFF;
    buff[1] = (length >> 8) & 0xFF;
    buff[2] = (length >> 16) & 0xFF;
    buff[3] = (length >> 24) & 0xFF;

    // Write
    flash_range_program(PROGRAM_LENGTH_OFFSET, buff, FLASH_PAGE_SIZE);

    // Wait for 'p' from host to signal the start of the program data
    do {
        ack = getchar();
    } while (ack != 'p');

    long int written = 0;
    char writing;
    while (written < length) {
        writing = (length - written > FLASH_PAGE_SIZE ? FLASH_PAGE_SIZE : length - written);
        // Get data
        for (int i = 0; i < writing; i++) {
            buff[i] = getchar();
        }
        // Write data
        flash_range_program(PROGRAM_OFFSET + written, buff, FLASH_PAGE_SIZE);

        // Clear buffer
        for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
            buff[i] = 0x00;
        }

        // update
        written += FLASH_PAGE_SIZE;
    }
}

void read_out() {
    uint32_t* length_ptr = (uint32_t *) PROGRAM_LENGTH_OFFSET;
    uint8_t* program_ptr = (uint8_t *) PROGRAM_OFFSET;

    for (int i = 0; i < *length_ptr; i++) {
        if (i % 16 == 15) {
            putchar('\n');
        }
        // print out each byte as a hexadecimal with an extra space between each one for readability
        printf("%hhx ", program_ptr[i]);
    }
}