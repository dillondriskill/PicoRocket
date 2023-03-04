/**
 * @file kernel.c
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief Contains the code for the kernel functions
 * @version 0.1
 * @date 2023-03-03
 * 
 * @copyright Copyright (c) 2023
 * 
*/

#include "pico/stdlib.h"

void initialize() {
    stdio_init_all();
    gpio_pull_up(PICO_DEFAULT_LED_PIN);

    /**
     * @todo Fuck with the clock
     * 
    */
}