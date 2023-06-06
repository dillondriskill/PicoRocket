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
#define ZERO_DEG_MILS 1000
#define MAX_DEG_MILS 2000
#define SERVO1 6
#define SERVO2 7
#define SERVO3 8
#define SERVO4 9
#define SERVORANGE 45

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
 * @brief connects to the host tools
 * 
*/
void host_tools();

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
 * @brief Set the servo on servoPin to the desired angle, 0 is facing forward
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
