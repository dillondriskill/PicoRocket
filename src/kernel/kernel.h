/**
 * @file kernel.h
 * @author Dillon Driskill (dillondriskill2@gmail.com)
 * @brief Contains headers for kernel.c
 * @version 0.1
 * @date 2023-03-03
 *
 * @copyright Copyright (c) 2023
 *
*/

#define LAST_FLASH_PAGE (XIP_BASE + PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define PROGRAM_LENGTH_OFFSET (LAST_FLASH_PAGE + 1)
#define PROGRAM_OFFSET (XIP_BASE + (512 * 1024))

/**
 * @brief Initializes the system. Starts by initializing all the io, and turning on the led.
 *
*/
void initialize();

/**
 * @brief Restarts the pico
 *
*/
void reset();

/**
 * @brief Resets the system into usb storage mode
 *
*/
void boot_reset();

/**
 * @brief Launches the rocket, starts executing guidance program
 *
*/
void launch_rocket();

/**
 * @brief Loads a guidance program into memory. Guidance programs are sequences of 
 *
*/
void load_program();

/**
 * @brief Reads out the program data
 * 
*/
void read_out();
