// icm42670_i2c.c
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define I2C_PORT i2c0
#define ICM42670_ADDR 0x69

// Mutex for FreeRTOS thread safety
#include "FreeRTOS.h"
#include "semphr.h"

#include <shellSDK/imu_icm42670.h>
static SemaphoreHandle_t i2c_mutex = NULL;

void i2c_init_interface() {
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);
    
    // Create mutex if using FreeRTOS
    i2c_mutex = xSemaphoreCreateMutex();
}

// Arduino-style register write
uint8_t writeRegister(uint8_t reg, uint8_t value) {
    if(xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t buffer[2] = {reg, value};
        int result = i2c_write_blocking(I2C_PORT, ICM42670_ADDR, buffer, 2, false);
        xSemaphoreGive(i2c_mutex);
        return (result == 2) ? 0 : 1;
    }
    return 1;
}

// Arduino-style register read
uint8_t readRegister(uint8_t reg) {
    if(xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t value;
        i2c_write_blocking(I2C_PORT, ICM42670_ADDR, &reg, 1, true); // Repeated start
        i2c_read_blocking(I2C_PORT, ICM42670_ADDR, &value, 1, false);
        xSemaphoreGive(i2c_mutex);
        return value;
    }
    return 0xFF;
}

// Burst read for sensor data
void readRegisters(uint8_t reg, uint8_t* data, size_t length) {
    if(xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        i2c_write_blocking(I2C_PORT, ICM42670_ADDR, &reg, 1, true); // Repeated start
        i2c_read_blocking(I2C_PORT, ICM42670_ADDR, data, length, false);
        xSemaphoreGive(i2c_mutex);
    }
}