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

#define ACCEL_ADDR  0x00 // 19
#define MAG_ADDR    0x00 // 1e
#define GYRO_ADDR   0b1101001

#define ACCEL_X     0x14
#define ACCEL_Y     0x54
#define ACCEL_Z     0x34

#define MAG_X       0xC0
#define MAG_Y       0xA0
#define MAG_Z       0xE0

#define GYRO_X      0x28
#define GYRO_Y      0x2A
#define GYRO_Z      0x2C

#define ACCEL_CONTROL_REG   0x04
#define ACCEL_CONTROL_BYTE  0b11100000
#define ACCEL_CONFIG_REG    0xC4
#define ACCEL_CONFIG_BYTE   0b00001100

#define MAG_CONTROL_REG     0x80
#define MAG_CONTROL_BYTE    0b00000111

#define GYRO_CONTROL_REG    0x20
#define GYRO_CONTROL_BYTE   0b00001111
#define GYRO_CONFIG_REG     0x23
#define GYRO_CONFIG_BYTE    0b00000000

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
