/**
 * @file rocket.c
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief The main entry point for the PicoRocket guidance computer
 * @version 0.1
 * @date 2023-03-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "rocket.h"
#include "../terminal/terminal.h"
#include "../kernel/kernel.h"

 /**
  * @brief PicoRocket entry point
  */
void main() {
    initialize();

    // Connect to host
    connect();

    return;
}
