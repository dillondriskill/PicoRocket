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
#include "../guidance/guidance.h"
#include "../kernel/kernel.h"
#pragma once

// Number of commands not including the help command. Needs to be updated every time a new command is made, as well as updating the code in commands.c
// This will be improved upon later
#define NUMBER_OF_COMMANDS 3

/**
 * @brief Struct of a terminal command. Contains the character that calls this command, as well as a pointer to the commands entry point
 *
*/
struct Command {
    char callerchar;
    void (*entry)(void);
    char helpmsg[64];
} commands[NUMBER_OF_COMMANDS];

struct Command guidance_o = {
    .callerchar = 'g',
    .entry = &guidance_entry,
    .helpmsg = "Runs the rockets guidance program\n",
};

struct Command reset_o = {
    .callerchar = 'r',
    .entry = &reset,
    .helpmsg = "Restarts the guidance computer\n"
};

struct Command reset_boot_o = {
    .callerchar = 'b',
    .entry = &reset,
    .helpmsg = "Restarts the pico into usb bootloader mode\n"
};
