/**
 * @file kernel.h
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief Contains headers for terminal.c
 * @version 0.1
 * @date 2023-03-03
 *
 * @copyright Copyright (c) 2023
 *
*/

/**
 * @brief Move cursor to x, y
 * 
 * @param x 
 * @param y 
*/
void movcurs(char x, char y);

/**
 * @brief Starts the terminal program. Will check for an open serial connection on the usb port
 * every 100 milliseconds, and will connect to that when it connects.
*/
void start_terminal();

/**
 * @brief Clears screen
 *
*/
void cls();

/**
 * @brief Outputs the 'beep' character. Will make the terminal beep
 *
*/
void beep();

/**
 * @brief Enables the terminal cursor
 * 
*/
void enablecurs();


/**
 * @brief Disables the terminal cursor
 * 
*/
void disablecurs();
