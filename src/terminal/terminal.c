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

#include "pico/stdlib.h"

void start_terminal() {
    bool connected = false;
    do {
        sleep_ms(100);
        connected = stdio_usb_connected();
    } while (connected = false);

    
}