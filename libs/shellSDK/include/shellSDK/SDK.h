/*
 * SDK V1.0.1
 * Copyright (C) 2024.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * 1 tab == 4 spaces!
 */
#ifndef SDK_H
#define SDK_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* I2C Related */
#define DEFAULT_I2C_SDA_PIN                     12
#define DEFAULT_I2C_SCL_PIN                     13

/* UART Related */
#define DEFAULT_UART_0                          0
#define DEFAULT_UART_1                          1

/* Switch Related */
#define SW1_PIN                                 02
#define SW2_PIN                                 22

/* LED Related */
#define RED_LED_PIN                             14

/* RGB LED Related */
#define RGB_LED_R                               18
#define RGB_LED_G                               19
#define RGB_LED_B                               20

/* Buzzer Related */
#define BUZZER_PIN                              17

/* Microphone Related */
#define PDM_DATA                                16
#define PDM_CLK                                 15

/* IMU Related */
#define ICM42670_I2C_ADDRESS               0x69
#define ICM42670_REG_WHO_AM_I              0x75
#define ICM42670_WHO_AM_I_RESPONSE         0x67
#define ICM42670_INT_CONFIG                0x06
#define ICM42670_INT1_CONFIG_VALUE         0x02  // Active High, Pulsed, Push-Pull

#define ICM42670_MAX_READ_LENGTH           256
#define ICM42670_MAX_WRITE_LENGTH          256

#define ICM42670_ACCEL_CONFIG0_REG         0x21
#define ICM42670_PWR_MGMT0_REG             0x1F

// FSR (Full Scale Range) Encoding
#define ICM42670_ACCEL_FSR_2G              0x03
#define ICM42670_ACCEL_FSR_4G              0x02
#define ICM42670_ACCEL_FSR_8G              0x01
#define ICM42670_ACCEL_FSR_16G             0x00

// ODR (Output Data Rate) Encoding for LN Mode (bit[3:0] of ACCEL_CONFIG0)
#define ICM42670_ACCEL_ODR_25HZ            0x0B
#define ICM42670_ACCEL_ODR_50HZ            0x0A
#define ICM42670_ACCEL_ODR_100HZ           0x09
#define ICM42670_ACCEL_ODR_200HZ           0x08
#define ICM42670_ACCEL_ODR_400HZ           0x07
#define ICM42670_ACCEL_ODR_800HZ           0x06
#define ICM42670_ACCEL_ODR_1600HZ          0x05

// LN Mode setting for accelerometer (bit[1:0] of PWR_MGMT0)
#define ICM42670_ACCEL_MODE_LN             0x03


#define ICM42670_GYRO_CONFIG0_REG          0x20
#define ICM42670_PWR_MGMT0_REG             0x1F

// FSR Encoding
#define ICM42670_GYRO_FSR_250DPS           0x03
#define ICM42670_GYRO_FSR_500DPS           0x02
#define ICM42670_GYRO_FSR_1000DPS          0x01
#define ICM42670_GYRO_FSR_2000DPS          0x00

// ODR Encoding
#define ICM42670_GYRO_ODR_25HZ             0x0B
#define ICM42670_GYRO_ODR_50HZ             0x0A
#define ICM42670_GYRO_ODR_100HZ            0x09
#define ICM42670_GYRO_ODR_200HZ            0x08
#define ICM42670_GYRO_ODR_400HZ            0x07
#define ICM42670_GYRO_ODR_800HZ            0x06
#define ICM42670_GYRO_ODR_1600HZ           0x05

// Gyro LN mode: bits [3:2] = 0b11 = 0x0C
#define ICM42670_GYRO_MODE_LN              0x0C

#define ICM42670_SENSOR_DATA_START_REG     0x09






// #define ICM42670_REG_PWR_MGMT0             0x0F
// #define ICM42670_REG_SIGNAL_PATH_RESET     0x4B
// #define ICM42670_REG_BANK_SEL              0x76

// #define ICM42670_SOFT_RESET_CMD            0x01
// #define ICM42670_PWR_ON_ACCEL_GYRO         0x0F  // LN mode both accel & gyro
// #define ICM42670_REG_ACCEL_CONFIG0         0x50
// #define ICM42670_REG_ACCEL_CONFIG1         0x51
// #define ICM42670_REG_GYRO_CONFIG0          0x52
// #define ICM42670_REG_GYRO_CONFIG1          0x53
// #define ICM42670_REG_ACCEL_DATA_X1         0x1F  // 6 bytes total: X1,X0,Y1,Y0,Z1,Z0
// #define ICM42670_REG_GYRO_DATA_X1          0x25  // 6 bytes total: X1,X0,Y1,Y0,Z1,Z0
// #define ICM42670_REG_TEMP_DATA1            0x1D  // 2 bytes
// #define ICM42670_REG_INT_STATUS            0x2D
// #define ICM42670_DRDY_BIT_MASK             0x01  // Bit 0 = Data Ready
// #define ICM42670_ACCEL_ODR_1KHZ_FS_16G     0x48  // FS_SEL = ±16g, ODR = 1kHz
// #define ICM42670_REG_INT_STATUS2           0x3F
// #define ICM42670_BIT_ACCEL_DRDY            0x04







/* Light Sensor Related */
#define VEML6030_I2C_ADDR                               0x10
#define VEML6030_INTERRUPT                              9
#define VEML6030_CONFIG_REG                             0x00
#define VEML6030_ALS_REG                                0x04

/* Humidity and Temperature Related */
#define HDC2021_I2C_ADDRESS                             0x40
#define HDC2021_INTERRUPT                               21
#define HDC2021_TEMP_LOW                                0x00
#define HDC2021_TEMP_HIGH                               0x01
#define HDC2021_HUMIDITY_LOW                            0x02
#define HDC2021_HUMIDITY_HIGH                           0x03
#define HDC2021_CONFIG                                  0x0E
#define HDC2021_MEASUREMENT_CONFIG                      0x0F
#define HDC2021_TEMP_THR_L                              0x13
#define HDC2021_TEMP_THR_H                              0x14
#define HDC2021_HUMID_THR_L                             0x15
#define HDC2021_HUMID_THR_H                             0x16


#endif // SDK_H