/*
Version 0.8

MIT License

Copyright (c) 2025 Raisul Islam, Iván Sánchez Milara

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef SDK_H
#define SDK_H


#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "pdm_microphone.h"   // pdm_samples_ready_handler_t
#include "pins.h"



/* =========================
 *  VEML6030
 * ========================= */
#define VEML6030_I2C_ADDR                       0x10
#define VEML6030_CONFIG_REG                     0x00
#define VEML6030_ALS_REG                        0x04

/* =========================
 *  HDC2021
 * ========================= */
#define HDC2021_I2C_ADDRESS                     0x40
#define HDC2021_TEMP_LOW                        0x00
#define HDC2021_TEMP_HIGH                       0x01
#define HDC2021_HUMIDITY_LOW                    0x02
#define HDC2021_HUMIDITY_HIGH                   0x03
#define HDC2021_CONFIG                          0x0E
#define HDC2021_MEASUREMENT_CONFIG              0x0F
#define HDC2021_TEMP_THR_L                      0x13
#define HDC2021_TEMP_THR_H                      0x14
#define HDC2021_HUMID_THR_L                     0x15
#define HDC2021_HUMID_THR_H                     0x16

/* =========================
 *  SSD1306
 * ========================= */

 #define SSD1306_I2C_ADDRESS                    0x3C

 /* =========================
 *  MEMS MICROPHONE
 * ========================= */

# define MEMS_SAMPLING_FREQUENCY                8000
# define MEMS_BUFFER_SIZE                       256

/* =========================
 *  ICM42670
 * ========================= */
#define ICM42670_I2C_ADDRESS                    0x69
#define ICM42670_I2C_ADDRESS_ALT                0x69
#define ICM42670_REG_WHO_AM_I                   0x75
#define ICM42670_WHO_AM_I_RESPONSE              0x67
#define ICM42670_INT_CONFIG                     0x06
#define ICM42670_INT1_CONFIG_VALUE              0x02
#define ICM42670_MAX_READ_LENGTH                256
#define ICM42670_MAX_WRITE_LENGTH               256

#define ICM42670_ACCEL_CONFIG0_REG              0x21
#define ICM42670_PWR_MGMT0_REG                  0x1F

// Accel FSR encodings
#define ICM42670_ACCEL_FSR_2G                   0x03
#define ICM42670_ACCEL_FSR_4G                   0x02
#define ICM42670_ACCEL_FSR_8G                   0x01
#define ICM42670_ACCEL_FSR_16G                  0x00
#define ICM42670_ACCEL_FSR_DEFAULT              4
#define ICM42670_GYRO_FSR_DEFAULT               250

// Accel ODR encodings (LN mode)
#define ICM42670_ACCEL_ODR_25HZ                 0x0B
#define ICM42670_ACCEL_ODR_50HZ                 0x0A
#define ICM42670_ACCEL_ODR_100HZ                0x09
#define ICM42670_ACCEL_ODR_200HZ                0x08
#define ICM42670_ACCEL_ODR_400HZ                0x07
#define ICM42670_ACCEL_ODR_800HZ                0x06
#define ICM42670_ACCEL_ODR_1600HZ               0x05
#define ICM42670_ACCEL_ODR_DEFAULT              100
#define ICM42670_GYRO_ODR_DEFAULT               100

// Accel LN mode in PWR_MGMT0
/* note: ICM42670_PWR_MGMT0_REG also defined above */
#define ICM42670_ACCEL_MODE_LN                  0x03
#define ICM42670_GYRO_CONFIG0_REG               0x20
#define ICM42670_REG_SIGNAL_PATH_RESET          0x02
#define ICM42670_RESET_CONFIG_BITS              0x10

// Gyro FSR encodings
#define ICM42670_GYRO_FSR_250DPS                0x03
#define ICM42670_GYRO_FSR_500DPS                0x02
#define ICM42670_GYRO_FSR_1000DPS               0x01
#define ICM42670_GYRO_FSR_2000DPS               0x00

// Gyro ODR encodings
#define ICM42670_GYRO_ODR_25HZ                  0x0B
#define ICM42670_GYRO_ODR_50HZ                  0x0A
#define ICM42670_GYRO_ODR_100HZ                 0x09
#define ICM42670_GYRO_ODR_200HZ                 0x08
#define ICM42670_GYRO_ODR_400HZ                 0x07
#define ICM42670_GYRO_ODR_800HZ                 0x06
#define ICM42670_GYRO_ODR_1600HZ                0x05

#define ICM42670_GYRO_MODE_LN                   0x0C
#define ICM42670_SENSOR_DATA_START_REG          0x09

/* =========================
 *  Public function prototypes
 * ========================= */

void init_hat_sdk(void);

/* Switches */
void init_sw1(void);
void init_sw2(void);

/* LEDs */
void init_red_led(void);
void toggle_red_led(void);
void set_red_led_status(bool);
void blink(int n);


void init_rgb_led(void);
void rgb_led_write(uint8_t r, uint8_t g, uint8_t b);
void stop_rgb_led();

/* Buzzer */
void init_buzzer(void);
void buzzer_play_tone(uint32_t frequency, uint32_t duration_ms);
void buzzer_turn_off(void);
void deinit_buzzer(void);

/* I2C generic helpers (use i2c_default) */
void i2c_init_default(uint sda_pin, uint scl_pin);
bool i2c_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop);
bool i2c_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop);

/* PDM microphone */
int  init_pdm_microphone(void);
int  init_microphone_sampling(void);
void end_microphone_sampling(void);
void pdm_microphone_set_callback(pdm_samples_ready_handler_t handler);
int  get_microphone_samples(int16_t *buffer, size_t samples);

/* Display helpers (SSD1306) */
void init_display(void);
void write_text(const char *text);
void write_text_xy(int16_t x0, int16_t y0, const char *text);
void set_text_cursor (int16_t x0, int16_t y0);
void draw_circle(int16_t x0, int16_t y0, int16_t r, bool fill);
void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void draw_square(uint32_t x, uint32_t y, uint32_t w, uint32_t h, bool fill);
void clear_display(void);
void display_stop(void);

/* VEML6030 */
void     veml6030_init(void);
uint32_t veml6030_read_light(void);
uint16_t _veml6030_read_register(uint8_t reg);
void veml6030_stop(void);

/* HDC2021 (public-facing subset;  */
void  hdc2021_set_low_temp_threshold(float temp);
void  hdc2021_set_high_temp_threshold(float temp);
void  hdc2021_set_high_humidity_threshold(float humid);
void  hdc2021_set_low_humidity_threshold(float humid);
float hdc2021_read_temperature(void);
float hdc2021_read_humidity(void);
void hdc2021_init(void);
void hdc2021_stop(void);

/* ICM42670 IMU (public-facing) */
int ICM42670_init(void);
int ICM42670_startAccel(uint16_t odr_hz, uint16_t fsr_g);
int ICM42670_startGyro(uint16_t odr_hz, uint16_t fsr_dps);
int ICM42670_enable_accel_gyro_ln_mode(void);
int ICM42670_read_sensor_data(float *ax, float *ay, float *az,
                              float *gx, float *gy, float *gz,
                              float *t);

#endif /* SDK_H */
