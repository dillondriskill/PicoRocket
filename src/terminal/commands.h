/**
 * @file commands.h
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief Contains the command structs and command list for the terminal program
 * @version 0.1
 * @date 2023-03-08
 *
 * @copyright Copyright (c) 2023
 *
*/

/* THIS SHOULD NEVER BE INCLUDED MORE THAN ONCE. THERE IS NO REASON TO INCLUDE THIS IN ANY FILE OTHER THAN terminal.c*/

#include "../guidance/guidance.h"
#include "../kernel/kernel.h"
#include "terminal.h"
#pragma once

// Maximum number of commands I'm assuming we will need
// Will be improved upon in the future

#define NUMBER_OF_COMMANDS 8

/**
 * @brief Struct of a terminal command. Contains the character that calls this command, as well as a pointer to the commands entry point
 *
*/
typedef struct {
    char callerchar;
    void (*entry)(void);
    char helpmsg[128];
}Command;

Command guidance_o = {
    .callerchar = 'g',
    .entry = &guidance_entry,
    .helpmsg = "Runs the rockets guidance program\n",
};

Command reset_o = {
    .callerchar = 'r',
    .entry = &reset,
    .helpmsg = "Restarts the guidance computer\n"
};

Command reset_boot_o = {
    .callerchar = 'b',
    .entry = &boot_reset,
    .helpmsg = "Restarts the guidance computer into usb bootloader mode\n"
};

Command load_o = {
    .callerchar = 'p',
    .entry = &load_program,
    .helpmsg = "Loads a guidance program onto flash. Should only be used by host tools\n"
};

Command launch_o = {
    .callerchar = 'l',
    .entry = &launch_rocket,
    .helpmsg = "Launches the rocket, then starts the guidance program\n"
};

Command read_o = {
    .callerchar = 'd',
    .entry = &read_out,
    .helpmsg = "Reads out the program data\n"
};

Command test_o = {
    .callerchar = 't',
    .entry = &test_flash,
    .helpmsg = "Tests the flash\n"
};

Command test_servo_o = {
    .callerchar = 's',
    .entry = &test_servo,
    .helpmsg = "Turns the servos to the middle of their range\n"
};

Command* commands[NUMBER_OF_COMMANDS] = {&guidance_o, &reset_boot_o, &reset_o, &load_o, &test_o, &launch_o, &read_o, &test_servo_o};
