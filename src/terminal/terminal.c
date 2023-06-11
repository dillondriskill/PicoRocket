/**
 * @file terminal.c
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief Contains code for the interface with a host tool or launcher
 * @version 0.1
 * @date 2023-03-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "tusb.h"
#include "terminal.h"
#include "../kernel/kernel.h"

void connect() {
    bool connected;
    char in;

    // Wait until a usb is connected
    do {
        sleep_ms(100);
        connected = stdio_usb_connected();
    } while (connected = false);

    while (!tud_cdc_connected()) {
        sleep_ms(100);
    }

    // Give the serial some time to connect before sending data out
    sleep_ms(1000);

    // Connect to host tools
    host_tools();

    reset();

}
