/**
 * @file rocket.c
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief The main code for the PicoRocket guidance computer
 * @version 0.1
 * @date 2023-03-03
 * 
 * @copyright Copyright (c) 2023
 * 
*/

#include "pico/stdlib.h"
#include "rocket.h"
#include "../kernel/kernel.h"
#include "../terminal/terminal.h"
#include "../guidance/guidance.h"

/**
 * @brief PicoRocket entry point
*/
void main() {
    initialize();

    start_terminal();

    return;
}