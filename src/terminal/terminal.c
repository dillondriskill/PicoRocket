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
#include "tusb.h"
#include "commands.h"
#include "terminal.h"
#include "../kernel/kernel.h"


static char get_command();
static bool do_command(const char command);
static void import_commands();

void start_terminal() {
    bool connected;
    bool prompting = true;
    char command;

    // actually be able to use the commands
    import_commands();


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
            putchar('\n');
            entered = true;
        } else {
            oldin = userin;
            if (oldin != 0) {
                putchar('\b');
            }
            putchar(oldin);
        }
    }

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
    bool found = false;

    // Special case for h
    if (command == 'h') {
        printf("Command | Meaning\n");
        for (int i = 0; i < NUMBER_OF_COMMANDS; i++) {
            printf("   %c    | %s\n", commands[i]->callerchar, commands[i]->helpmsg);

        }
    } else {
        for (int i = 0; i < NUMBER_OF_COMMANDS; i++) {
            if (commands[i]->callerchar == command) {
                (commands[i]->entry)();
                found = true;
            }
        }
    }
    if (!found) {
        printf("Unknown Command!\n");
    }
    return continuing;
}

void cls() {
    printf("\x1b[2J");
}

void beep() {
    putchar('\7');
}

/**
 * @brief Puts the commands from commands.h into an array. I will find a way to automate this better.
 *
*/
static void import_commands() {
    commands[0] = &guidance_o;
    commands[1] = &reset_o;
    commands[2] = &reset_boot_o;
    commands[3] = &load_o;
}
