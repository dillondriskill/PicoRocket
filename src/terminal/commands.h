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

#pragma once

// Number of commands not including the help command. Needs to be updated every time a new command is made
#define NUMBER_OF_COMMANDS 5

/**
 * @brief Struct of a terminal command. Contains the character that calls this command, as well as a pointer to the commands entry point
 *
*/
struct Command {
    char callerchar;
    void (*entry)(void);
    char* helpmsg;
} commands[NUMBER_OF_COMMANDS];