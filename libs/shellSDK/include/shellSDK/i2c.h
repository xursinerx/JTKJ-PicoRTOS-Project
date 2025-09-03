#pragma once

#include <stdint.h>
#include <stddef.h>


/**
 * @brief Initialize I2C0 interface (pins 4 = SDA, 5 = SCL at 400 kHz).
 * 
 * Creates a FreeRTOS mutex internally for thread safety.
 */
void i2c_init_interface(void);

/**
 * @brief Write a single byte to a device register.
 * 
 * @param reg   Register address
 * @param value Value to write
 * @return 0 on success, 1 on failure
 */
uint8_t writeRegister(uint8_t reg, uint8_t value);

/**
 * @brief Read a single byte from a device register.
 * 
 * @param reg Register address
 * @return Register value on success, 0xFF on failure
 */
uint8_t readRegister(uint8_t reg);

/**
 * @brief Read multiple consecutive registers.
 * 
 * @param reg   Starting register address
 * @param data  Destination buffer
 * @param length Number of bytes to read
 */
void readRegisters(uint8_t reg, uint8_t* data, size_t length);


