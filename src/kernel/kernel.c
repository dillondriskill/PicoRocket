/**
 * @file kernel.c
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief Contains the code for the kernel functions
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