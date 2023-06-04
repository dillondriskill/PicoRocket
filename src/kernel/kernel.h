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

#define PROGRAM_LENGTH_OFFSET (512 * 1024)
#define PROGRAM_OFFSET (513 * 1024)
#define OK 0x00
#define ZERODEGMILLIS 400
#define MAXDEGMNILLIS 2400
#define MILTODEG 0.09
#define DEGTOMIL (1000/90)

/**
 * @brief Initializes the system. Starts by initializing all the io, and turning on the led.
 *
*/
void initialize();

/**
 * @brief Restarts the pico using the watchdog timer
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

/**
 * @brief Tests the flash
 * 
*/
void test_flash();

/**
 * @brief Tests a servo on pin 0
 * 
*/
void test_servo();

/**
 * @brief puts a pwm signal of length millis on servoPin
 * 
 * @param servoPin the pin to set
 * @param millis the length in milliseconds of the pwm cycle
*/
void setMillis(int servoPin, uint millis);

/**
 * @brief Sets up a GPIO pin to be a servo
 * 
 * @param servoPin the pin to be set up
 * @param startMillis the milliseconds to start the pwm at
*/
void setServo(int servoPin, uint startMillis);

/**
 * @brief Set the servo on servoPin to the desired angle
 * 
 * @param servoPin servo to set
 * @param deg angle to set
*/
void setDeg(int servoPin, float deg);

/**
 * @brief Set up a GPIO pin to be a servo
 * 
 * @param servoPin pin to be set up
 * @param deg angle to start at
*/
void setServoDeg(int servoPin, float deg);
