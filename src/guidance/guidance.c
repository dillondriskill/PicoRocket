/**
 * @file guidance.h
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief Contains code the the guidance program
 * @version 0.1
 * @date 2023-03-03
 *
 * @copyright Copyright (c) 2023
 *
*/

#include <stdio.h>

#include "guidance.h"
#include "../kernel/kernel.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include  "pico/time.h"
#include "hardware/flash.h"

// Follows the guidance program on core 1, should only be used on core 1, not core 0
void core1_entry() {

    uint8_t* length_ptr = (uint8_t*)XIP_BASE + PROGRAM_LENGTH_OFFSET;
    uint8_t* program_ptr = (uint8_t*)XIP_BASE + PROGRAM_OFFSET;
    uint32_t length = (*length_ptr) * FLASH_PAGE_SIZE;
    uint8_t* program_end = program_ptr + length;

    // Set up the wings to point forward
    setServoDeg(SERVO1, 45);
    setServoDeg(SERVO2, 45);
    setServoDeg(SERVO3, 45);
    setServoDeg(SERVO4, 45);

    // trigger the timer
    multicore_fifo_push_blocking(0x00);

    // Go until we reach the end of the program, or until we see 0xFF, in case there is stuff at the end, or whatever
    // every 4 bytes represents 1/100th of a second, and each byte holds the angle in degrees, from 0-90 of each servo
    while (*program_ptr != 0xFF) {
        
        setDeg(SERVO1, *program_ptr); program_ptr++;
        setDeg(SERVO2, *program_ptr); program_ptr++;
        setDeg(SERVO3, *program_ptr); program_ptr++;
        setDeg(SERVO4, *program_ptr); program_ptr++;
        
        sleep_ms(10);
    }

    // Tell the other core were done
    multicore_fifo_push_blocking(0x00);

}

void guidance_entry() {
    absolute_time_t endtime;
    absolute_time_t starttime;

    printf("Following guidance program...\n");

    // Reset everything to be safe
    multicore_reset_core1();
    multicore_fifo_drain();

    // Launch core 1 to follow the guidance program, and wait for that to send the all clear
    multicore_launch_core1(&core1_entry);

    multicore_fifo_pop_blocking();
    starttime = get_absolute_time();
    multicore_fifo_pop_blocking();
    endtime = get_absolute_time();

    uint64_t diff = absolute_time_diff_us(starttime, endtime);

    // Reset it
    multicore_reset_core1();
    printf("Guidance program complete in: %llu us\n", diff);

}
