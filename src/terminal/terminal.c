/**
 * @file terminal.h
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief Contains code for the terminal program
 * @version 0.1
 * @date 2023-03-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "../kernel/kernel.h"
#include "tusb.h"

static char get_command();
static bool do_command(const char command);

void start_terminal() {
    bool connected;
    bool prompting = true;
    char command;

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
    cls();
    beep();

    /**
     * 1) Put user prompt (output welcome message on first one)
     * 2) Get command
     * 3) Do command
     * 4) Repeat until a command requires exiting
     */
    printf("PicoRocket Guidance System V0.1\n");
    while (prompting) {
        printf(">   ");
        command = get_command();
        prompting = do_command(command);
    }

    printf("Exiting terminal program...\n");

    return;
}

/**
 * @brief Get the command from the user
 *
 * @return char The command character from the user
*/
static char get_command() {
    bool entered = false;
    char userin = 0;
    char oldin = 0;

    // Allow up to 255 characters to be typed, leaving room for the null terminator
    while (!entered) {
        userin = getchar();
        // Check for enter, and return the last character actually typed
        if (userin == '\r') {
            entered = true;
        } else {
            oldin = userin;
            if (oldin != 0) {
                putchar('\b');
            }
            putchar(oldin);
        }
    }
    putchar('\n');

    return oldin;
}

/**
 * @brief Take the command from the user and does it. Returns whether or not the terminal should continue after the command execution
 *
 * @param command the command character from the user
 * @return true Continue execution
 * @return false Terminate terminal
*/
static bool do_command(char command) {
    bool continuing = true;

    switch (command) {
    case 'q':
        continuing = false;
        break;
    case 'r':
        printf("Resetting...\n\n");
        sleep_ms(100);
        reset();
        continuing = false; // Will never execute this far
        break;
    case 'b':
        printf("Entering bootloader mode...\n\n");
        sleep_ms(100);
        boot_reset();
        continuing = false; // Will never execute:
        break;
    case 0:
        break;
    default:
        printf("Unknown command!\n\n");
        break;
    }

    return continuing;
}

void cls() {
    printf("\x1b[2J");
}

void beep() {
    putchar('\7');
}