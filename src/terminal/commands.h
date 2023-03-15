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
#pragma once

// Maximum number of commands I'm assuming we will need
// Will be improved upon in the future

#define NUMBER_OF_COMMANDS 6

/**
 * @brief Struct of a terminal command. Contains the character that calls this command, as well as a pointer to the commands entry point
 *
*/
typedef struct {
    char callerchar;
    void (*entry)(void);
    char helpmsg[64];
}Command;

Command* commands[NUMBER_OF_COMMANDS];

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
    .helpmsg = "Loads a guidance program onto flash"
};

Command launch_o = {
    .callerchar = 'l',
    .entry = &launch_rocket,
    .helpmsg = "Launches the rocket, then starts the guidance program"
};

Command read_o = {
    .callerchar = 'd',
    .entry = &read_out,
    .helpmsg = "Reads out the program data"
};
