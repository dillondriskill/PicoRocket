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
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "tusb.h"
#include "commands.h"
#include "terminal.h"
#include "../kernel/kernel.h"


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
     * 1) Page title
     * 2) Get command
     * 3) Do command
     * 4) Repeat until a command requires exiting
     */
    printf("\x1b\x63"); // resets terminal
    printf("\x1b[7mPicoRocket Guidance System V0.2                                                 \x1b[0m\n"); // highlighted title bar
    printf("\x1b[2;24;r"); // sets margins
    printf("\x1b[?4h"); // smooth scrolling
    enablecurs(); // make sure cursor is on


    // command loop
    while (prompting) {
        enablecurs();
        printf(">   ");
        command = get_command();
        prompting = do_command(command);
    }

    printf("Exiting terminal program...\n");
}

/**
 * @brief Get the command from the user
 *
 * @return char The command character from the user
*/
static char get_command() {
    bool entered = false;
    char userin = 0;
    char command = 0;

    while (!entered) {
        userin = getchar();
        // Check for enter, and return the last character actually typed
        if (userin == '\r') {
            putchar('\n');
            entered = true;
        } else {
            if isalpha(userin) {
                printf("\x1b[D%c", userin);
                command = userin;
            }
        }
    }

    return command;
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
        // go through the list of commands and see if we find one, if we do find it
        // use the function that was pointed at
        for (int i = 0; i < NUMBER_OF_COMMANDS; i++) {
            if (commands[i]->callerchar == command) {
                (commands[i]->entry)(); // function pointer
                found = true;
            }
        }
        // wasnt found
        if (!found) {
            printf("Unknown Command!\n");
        }
    }

    return continuing;
}

void cls() {
    movcurs(0, 2);
    printf("\x1b[0J");
}

void beep() {
    putchar('\7');
}

void movcurs(char x, char y) {
    printf("\x1b[%hhu;%hhuH", y, x);
}

void enablecurs() {
    printf("\x1b[?25h");
}

void disablecurs() {
    printf("\x1b[?25l");
}
