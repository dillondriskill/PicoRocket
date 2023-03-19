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

#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "hardware/flash.h"
#include "kernel.h"
#include "hardware/sync.h"


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
    flash_range_erase((uint32_t)PROGRAM_LENGTH_OFFSET, 4096*16);
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
    temp = (uint8_t *)(XIP_BASE + PROGRAM_LENGTH_OFFSET);
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
    uint8_t* length_ptr = (uint8_t *) XIP_BASE + PROGRAM_LENGTH_OFFSET;
    uint8_t* program_ptr = (uint8_t *) XIP_BASE + PROGRAM_OFFSET;

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
    sleep_ms(100);
    printf("Testing the flash...\n");

    for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
        buffer[i] = i;
    }

    printf("Buffer filled with: \n");
    for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
        if (i % 16 == 0) {
            printf("\n");
        }
        printf("%02hhX ", buffer[i]);
    }
    putchar('\n');

    printf("Programming...\n");
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(sector, FLASH_SECTOR_SIZE);
    flash_range_program(sector, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    printf("Flash programmed...\n\n");

    printf("The result is: \n");

    const uint8_t* written_data = (uint8_t *) (XIP_BASE + sector);

    for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
        if (i % 16 == 0) {
            printf("\n");
        }
        printf("%02hhX ", written_data[i]);
    }
    putchar('\n');
}
