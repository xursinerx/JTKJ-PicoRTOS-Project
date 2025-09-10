#include <shellSDK/SDK.h>
// #include <icm42670.h>

//#include "tusb.h" //is it needed?
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include <shellSDK/ssd1306.h>
#include <shellSDK/imu_icm42670.h>
#include <shellSDK/pdm_microphone.h>
#include <stdio.h>
#include <math.h>




//This trick tries to solve the leakage problem in rp2350 (Errata 9)
#if defined(PICO_RP2350)
    #include "hardware/gpio.h"

    static inline bool gpio_get_safe(uint pin) {
        hw_set_bits(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_IE_BITS); // enable input buffer
        __asm volatile("nop; nop; nop; nop;"); // short settle
        bool v = gpio_get(pin);// read
        hw_clear_bits(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_IE_BITS); // disable input buffer again
        //printf("Reading 2350");
        return v;
    }

    // Redirect any later uses of gpio_get(...) to our safe version on RP2350
    #define gpio_get(pin) gpio_get_safe((pin))
#endif

// By default these devices  are on bus address 0x68
static int addr = 0x68;
static ssd1306_t disp;


void init_shell(){
    //Turn off the RGB
    stop_rgb_led();
}

// Button related function
 void init_sw1() {
    // Initialize the button pin as an input with a pull-up resistor
    gpio_init(SW1_PIN);
    gpio_set_dir(SW1_PIN, GPIO_IN);

}

 void init_sw2() {
    // Initialize the button pin as an input with a pull-up resistor
    gpio_init(SW2_PIN);
    gpio_set_dir(SW2_PIN, GPIO_IN);
}

// LED related function
 void init_red_led() {
    // Initialize the LED pin as an output
    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);
}

void toggle_red_led() {
    bool curr = gpio_get(RED_LED_PIN);
    gpio_put(RED_LED_PIN, !curr);
}

// RGB related function
 void init_rgb_led() {
    // Initialize the PWM slice for each RGB pin
    gpio_set_function(RGB_LED_R, GPIO_FUNC_PWM);
    gpio_set_function(RGB_LED_G, GPIO_FUNC_PWM);
    gpio_set_function(RGB_LED_B, GPIO_FUNC_PWM);

    // Get the PWM slice numbers for each GPIO pin
    uint slice_num_r = pwm_gpio_to_slice_num(RGB_LED_R);
    uint slice_num_g = pwm_gpio_to_slice_num(RGB_LED_G);
    uint slice_num_b = pwm_gpio_to_slice_num(RGB_LED_B);

    // Set PWM clock to a frequency suitable for LED control
    pwm_set_clkdiv(slice_num_r, 4.0f);
    pwm_set_clkdiv(slice_num_g, 4.0f);
    pwm_set_clkdiv(slice_num_b, 4.0f);

    // Enable PWM outputs
    pwm_set_enabled(slice_num_r, true);
    pwm_set_enabled(slice_num_g, true);
    pwm_set_enabled(slice_num_b, true);
}

//RGB off
void stop_rgb_led(){
     // Stop PWM on those slices (optional)
    pwm_set_enabled(pwm_gpio_to_slice_num(RGB_LED_R), false);
    pwm_set_enabled(pwm_gpio_to_slice_num(RGB_LED_G), false);
    pwm_set_enabled(pwm_gpio_to_slice_num(RGB_LED_B), false);

    // Return pins to GPIO input (Hi-Z)
    gpio_set_function(RGB_LED_R, GPIO_FUNC_SIO);
    gpio_set_dir(RGB_LED_R, GPIO_IN);
    gpio_disable_pulls(RGB_LED_R);

    gpio_set_function(RGB_LED_G, GPIO_FUNC_SIO);
    gpio_set_dir(RGB_LED_G, GPIO_IN);
    gpio_disable_pulls(RGB_LED_G);

    gpio_set_function(RGB_LED_B, GPIO_FUNC_SIO);
    gpio_set_dir(RGB_LED_B, GPIO_IN);
    gpio_disable_pulls(RGB_LED_B);
}

//Channel active to low level (common anode). We need to invert the value
 void rgb_led_write(uint8_t r, uint8_t g, uint8_t b) {

    //Invert the values
    r = 255-r;
    g = 255-g;
    b = 255-b;
    // Get the PWM slice numbers for each GPIO pin
    uint slice_num_r = pwm_gpio_to_slice_num(RGB_LED_R);
    uint slice_num_g = pwm_gpio_to_slice_num(RGB_LED_G);
    uint slice_num_b = pwm_gpio_to_slice_num(RGB_LED_B);

    // Convert the 0-255 value to a 16-bit duty cycle (0-65535)
    uint16_t r_value = r * r; // Mapping 0-255 to 0-65535 (linearly)
    uint16_t g_value = g * g; // Mapping 0-255 to 0-65535 (linearly)
    uint16_t b_value = b * b; // Mapping 0-255 to 0-65535 (linearly)

    // Set the PWM level for each color
    pwm_set_gpio_level(RGB_LED_R, r_value);
    pwm_set_gpio_level(RGB_LED_G, g_value);
    pwm_set_gpio_level(RGB_LED_B, b_value);
}


// Buzzer-related functions
 void init_buzzer() {
    // Initialize the buzzer pin as an output
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
}

 void buzzer_play_tone(uint32_t frequency, uint32_t duration_ms) {
    // Calculate the period (in microseconds) and number of cycles
    uint32_t period_us = 1000000 / frequency;
    uint32_t num_cycles = duration_ms * frequency / 1000;

    // Generate the tone by toggling the buzzer pin at the specified frequency
    for (uint32_t i = 0; i < num_cycles; i++) {
        gpio_put(BUZZER_PIN, 1);
        busy_wait_us(period_us / 2);
        gpio_put(BUZZER_PIN, 0);
        busy_wait_us(period_us / 2);
    }
}

 void buzzer_turn_off() {
    // Turn off the buzzer by setting the pin to low
    gpio_put(BUZZER_PIN, 0);
}

void deinit_buzzer() {
    // Deinitialize the buzzer pin
    gpio_deinit(BUZZER_PIN);
}


// Initialize I2C peripheral
void i2c_init_default(uint sda_pin, uint scl_pin) {
    i2c_init(i2c_default, 400*1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

// Generic I2C write function
bool i2c_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop) {
    int bytes_written = i2c_write_blocking(i2c_default, addr, src, len, nostop);
    return bytes_written == (int)len;
}

// Generic I2C read function
bool i2c_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop) {
    int bytes_read = i2c_read_blocking(i2c_default, addr, dst, len, nostop);
    return bytes_read == (int)len;
}


// Microphone related functions
 int init_pdm_microphone() {
    const struct pdm_microphone_config config = {
    // GPIO pin for the PDM DAT signal
    .gpio_data = PDM_DATA,

    // GPIO pin for the PDM CLK signal
    .gpio_clk = PDM_CLK,

    // PIO instance to use
    .pio = pio0,

    // PIO State Machine instance to use
    .pio_sm = 0,

    // sample rate in Hz
    .sample_rate = 16000,

    // number of samples to buffer
    .sample_buffer_size = 256,
    };

    return pdm_microphone_init(&config);
   
}

 int start_pdm_microphone(){
    return pdm_microphone_start();
}

 void stop_pdm_microphone(){
    pdm_microphone_stop();
}

void pdm_microphone_set_callback(pdm_samples_ready_handler_t handler) {
    pdm_microphone_set_samples_ready_handler(handler);
}

int pdm_microphone_read_data(int16_t* buffer, size_t samples) {
    return pdm_microphone_read(buffer,samples);
}




// Display-related functions
 void init_display() {
    // Initialize the SSD1306 display with external VCC
    disp.external_vcc = false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c_default);

    // Clear the display
    ssd1306_clear(&disp);
}

 void write_text(const char *word) {
    // Clear the display
    ssd1306_clear(&disp);

    // Draw the text at the specified position with a font size of 2
    ssd1306_draw_string(&disp, 8, 24, 2, word);

    // Update the display
    ssd1306_show(&disp);

    // Delay for 800 milliseconds
    sleep_ms(800);
}

 void draw_circle(int16_t x0, int16_t y0, int16_t r) {
    // Draw a circle using the Bresenham algorithm
    int16_t x = r - 1;
    int16_t y = 0;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * x;
    int16_t ddF_x2 = 0;
    int16_t ddF_y2 = -2 * x;

    while (x >= y) {
        // Draw the eight symmetrical points of the circle
        ssd1306_draw_pixel(&disp, x0 + x, y0 + y);
        ssd1306_draw_pixel(&disp, x0 + y, y0 + x);
        ssd1306_draw_pixel(&disp, x0 - y, y0 + x);
        ssd1306_draw_pixel(&disp, x0 - x, y0 + y);
        ssd1306_draw_pixel(&disp, x0 + x, y0 - y);
        ssd1306_draw_pixel(&disp, x0 + y, y0 - x);
        ssd1306_draw_pixel(&disp, x0 - y, y0 - x);
        ssd1306_draw_pixel(&disp, x0 - x, y0 - y);

        // Update the error terms and increment/decrement x and y accordingly
        if (ddF_y > 0) {
            x--;
            ddF_x2 += 2;
            ddF_y += 2;
        }
        ddF_y += 2;
        y++;
        ddF_x += 2;
    }
}

 void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    // Draw a line between the specified points
    ssd1306_draw_line(&disp, x0, y0, x1, y1);

    // Update the display
    ssd1306_show(&disp);
}

 void draw_square(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    // Draw a square at the specified position with the given width and height
    ssd1306_draw_square(&disp, x, y, w, h);

    // Update the display
    ssd1306_show(&disp);
}

 void clear_display() {
    // Clear the display
    ssd1306_clear(&disp);
}


// Light sensor related function
// Useful info at: https://learn.sparkfun.com/tutorials/qwiic-ambient-light-sensor-veml6030-hookup-guide/all#arduino-library
// Programming application: https://www.vishay.com/docs/84367/designingveml6030.pdf
// Datasheet: https://www.vishay.com/docs/84366/veml6030.pdf
void veml6030_init() {
    // Configure sensor settings (100ms integration time, gain 1/8, power on)
    //Bit 12:11 = 10 (gain1/8)
    //Bit 9:6 = 0000 (Integration time 100ms)
    //Bit 5:4 Persisentec protect number setting (00 -> 1) 
    //Bit 1 =0 INT disable
    //Bit 0 = 0 Power on
    // 0b0001 0000 0000 0000 -> =0x1000

    uint8_t config[3] = {
        VEML6030_CONFIG_REG,  // Configuration register
        0x00,                 // High byte: Gain 1/8 (00), reserved bits
        0x10                  // Low byte: 100ms integration time (010 in bits 6-8), power on (bit 0 = 0)
    };
    
    // Write configuration to sensor
    i2c_write_blocking(i2c_default, VEML6030_I2C_ADDR, config, sizeof(config), false);
    sleep_ms(10);
}

// Read light level from VEML6030
// Ligt in LUX
// Note: sampling time should be > IT -> in this case it has been 100ms by defintion. 
uint32_t veml6030_read_light() {
    uint8_t reg = VEML6030_ALS_REG;
    uint8_t data[2] = {0,0};

    // Select ALS output register
    i2c_write_blocking(i2c_default, VEML6030_I2C_ADDR, &reg, 1, true);
    // Read two bytes (MSB first)
    i2c_read_blocking(i2c_default, VEML6030_I2C_ADDR, data, sizeof(data), false);
    //data [0] contains the LSB and data[1] the MSB
    uint16_t lightbits = ((uint16_t)data[0]) |((uint16_t) data[1]<<8);
    // See table page 5 https://www.vishay.com/docs/84367/designingveml6030.pdf
    // With other values of gain (1/8) and integration time (100ms) 
    uint32_t luxVal_uncorrected = lightbits *  0.5376;
    //NOT SURE IF THIS IS CORRECT JUST CHECK. 
    if (luxVal_uncorrected>1000){
        // Polynomial is pulled from pg 10 of the datasheet. 
        // See https://github.com/sparkfun/SparkFun_Ambient_Light_Sensor_Arduino_Library/blob/efde0817bd6857863067bd1653a2cfafe6c68732/src/SparkFun_VEML6030_Ambient_Light_Sensor.cpp#L409
        uint32_t luxVal = (.00000000000060135 * (pow(luxVal_uncorrected, 4))) - 
                            (.0000000093924 * (pow(luxVal_uncorrected, 3))) + 
                            (.000081488 * (pow(luxVal_uncorrected,2))) + 
                            (1.0023 * luxVal_uncorrected);
        return luxVal;
    }
    return  luxVal_uncorrected;
}



//This method might be utility method in the future.
uint16_t _veml6030_read_register(uint8_t reg) {
    uint8_t data[2] = {0,0};

    // Select ALS output register
    i2c_write_blocking(i2c_default, VEML6030_I2C_ADDR, &reg, 1, true);
    // Read two bytes (MSB first)
    i2c_read_blocking(i2c_default, VEML6030_I2C_ADDR, data, sizeof(data), false);
    //data [0] contains the LSB and data[1] the MSB
    return ((uint16_t)data[0]) |((uint16_t) data[1]<<8);
}

void stop_veml6030(){
    uint8_t config[3] = {
        VEML6030_CONFIG_REG,  // Configuration register
        0x00,                 // High byte: Gain 1/8 (00), reserved bits
        0x11                  // Low byte: 100ms integration time (010 in bits 6-8), power on (bit 0 = 0). Power off (last bit to 1)
    };
    
    // Write configuration to sensor
    i2c_write_blocking(i2c_default, VEML6030_I2C_ADDR, config, sizeof(config), false);
    sleep_ms(10);
}




// Temperature & Humidity sensor related function
// https://www.ti.com/lit/ds/symlink/hdc2021.pdf?ts=1757522824481&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FHDC2021
// https://www.ti.com/lit/ug/snau250/snau250.pdf?ts=1757438909914

 static int8_t read_hdc2021_register(uint8_t reg) {
    uint8_t data;
    i2c_write(HDC2021_I2C_ADDRESS, &reg, 1, true);
    i2c_read(HDC2021_I2C_ADDRESS, &data, 1, false);
    return data;
}

 static void write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write(HDC2021_I2C_ADDRESS, data, sizeof(data), false);
}

 static void hdc2021_reset() {
    uint8_t configContents = read_hdc2021_register(HDC2021_CONFIG);
    write_register(HDC2021_CONFIG, configContents | 0x80);
    sleep_ms(50);
}

static void hdc2021_setMeasurementMode() {
    uint8_t configContents = read_hdc2021_register(HDC2021_MEASUREMENT_CONFIG);
    write_register(HDC2021_MEASUREMENT_CONFIG, configContents & 0xF9);
}

static void hdc2021_setRate() {
    uint8_t configContents = read_hdc2021_register(HDC2021_CONFIG);
    configContents = (configContents & 0x8F) | 0x50; // Set 1 measurement/second
    write_register(HDC2021_CONFIG, configContents);
}

static void hdc2021_setTempRes() {
    uint8_t configContents = read_hdc2021_register(HDC2021_MEASUREMENT_CONFIG);
    write_register(HDC2021_MEASUREMENT_CONFIG, configContents & 0x3F); // 14-bit
}

static void hdc2021_setHumidityRes() {
    uint8_t configContents = read_hdc2021_register(HDC2021_MEASUREMENT_CONFIG);
    write_register(HDC2021_MEASUREMENT_CONFIG, configContents & 0xCF); // 14-bit
}

 static void hdc2021_triggerMeasurement() {
    uint8_t configContents = read_hdc2021_register(HDC2021_MEASUREMENT_CONFIG);
    write_register(HDC2021_MEASUREMENT_CONFIG, configContents | 0x01);
}

void hdc2021_set_low_temp_threshold(float temp) {
    temp = (temp < -40.0f) ? -40.0f : (temp > 125.0f) ? 125.0f : temp;
    uint8_t temp_thresh = (uint8_t)((temp + 40.0f) * 256.0f / 165.0f);
    write_register(HDC2021_TEMP_THR_L, temp_thresh);
}

void hdc2021_set_high_temp_threshold(float temp) {
    temp = (temp < -40.0f) ? -40.0f : (temp > 125.0f) ? 125.0f : temp;
    uint8_t temp_thresh = (uint8_t)((temp + 40.0f) * 256.0f / 165.0f);
    write_register(HDC2021_TEMP_THR_H, temp_thresh);
}

void hdc2021_set_high_humidity_threshold(float humid) {
    humid = (humid < 0.0f) ? 0.0f : (humid > 100.0f) ? 100.0f : humid;
    uint8_t humid_thresh = (uint8_t)(humid * 2.56f);
    write_register(HDC2021_HUMID_THR_H, humid_thresh);
}

void hdc2021_set_low_humidity_threshold(float humid) {
    humid = (humid < 0.0f) ? 0.0f : (humid > 100.0f) ? 100.0f : humid;
    uint8_t humid_thresh = (uint8_t)(humid * 2.56f);
    write_register(HDC2021_HUMID_THR_L, humid_thresh);
}
// By default it sets following modes: 
// Measurement methods: Temp + Measurement
// Sampling rate: 1 Hz
// Temperature resolution: 14 bits
// Humidity resolution: 14 bits
// It triggers continous measurements. 
 void hdc2021_init() {
    hdc2021_reset();
    hdc2021_set_high_temp_threshold(50);
    hdc2021_set_low_temp_threshold(-30);
    hdc2021_set_high_humidity_threshold(100);
    hdc2021_set_low_humidity_threshold(0);
    hdc2021_setMeasurementMode();
    hdc2021_setRate();
    hdc2021_setTempRes();
    hdc2021_setHumidityRes();
    hdc2021_triggerMeasurement();
}

// Note that sampling rate is 1Hz
float hdc2021_read_temperature() {
    uint8_t reg = HDC2021_TEMP_LOW;
    uint8_t data[2];
    
    i2c_write(HDC2021_I2C_ADDRESS, &reg, 1, true);
    i2c_read(HDC2021_I2C_ADDRESS, data, 2, false);
    uint16_t raw = ((uint16_t) data[1] << 8) | data[0];
    return (raw * 165.0f / 65536.0f) - 40.0f;
}

//Note that sampling rate is 1 HX
float hdc2021_read_humidity() {
    uint8_t reg = HDC2021_HUMIDITY_LOW;
    uint8_t data[2];
    
    i2c_write(HDC2021_I2C_ADDRESS, &reg, 1, true);
    i2c_read(HDC2021_I2C_ADDRESS, data, 2, false);
    
    uint16_t raw = ((uint16_t) data[1] << 8) | data[0];
    return (raw * 100.0f / 65536.0f);
}


// IMU related function

 int icm_i2c_write_byte(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    int result = i2c_write_blocking(i2c_default, ICM42670_I2C_ADDRESS, buf, 2, false);
    return result == 2 ? 0 : -1;
}

// helper to read a byte from a register
 int icm_i2c_read_byte(uint8_t reg, uint8_t *value) {
    int result = i2c_write_blocking(i2c_default, ICM42670_I2C_ADDRESS, &reg, 1, true);
    if (result != 1) return -1;
    result = i2c_read_blocking(i2c_default, ICM42670_I2C_ADDRESS, value, 1, false);
    return result == 1 ? 0 : -1;
}

 int icm_i2c_read_bytes(uint8_t reg, uint8_t *buffer, uint8_t len) {
    int result = i2c_write_blocking(i2c_default, ICM42670_I2C_ADDRESS, &reg, 1, true);
    if (result != 1) return -1;
    result = i2c_read_blocking(i2c_default, ICM42670_I2C_ADDRESS, buffer, len, false);
    return result == len ? 0 : -2;
}


int ICM42670_init() {
    uint8_t who_am_i = 0;
    int rc = 0;

    // Step 1: Check WHO_AM_I
    rc = icm_i2c_read_byte(ICM42670_REG_WHO_AM_I, &who_am_i);
    if (rc != 0) {
        return -2;
    }
    if (who_am_i != ICM42670_WHO_AM_I_RESPONSE) {
        return -3;
    }

    // Step 2: Configure INT1 pin - polarity high, pulsed, push-pull
    rc = icm_i2c_write_byte(ICM42670_INT_CONFIG, ICM42670_INT1_CONFIG_VALUE);
    if (rc != 0) {
        return -4;
    }


    // Step 3: Success
    return 0;
}

int ICM42670_startAccel(uint16_t odr_hz, uint16_t fsr_g) {
    uint8_t fsr_bits = 0;
    uint8_t odr_bits = 0;

    // Map FSR to register bits
    switch (fsr_g) {
        case 2:  fsr_bits = ICM42670_ACCEL_FSR_2G; break;
        case 4:  fsr_bits = ICM42670_ACCEL_FSR_4G; break;
        case 8:  fsr_bits = ICM42670_ACCEL_FSR_8G; break;
        case 16: fsr_bits = ICM42670_ACCEL_FSR_16G; break;
        default: return -1; // invalid FSR
    }

    // Map ODR to register bits
    switch (odr_hz) {
        case 25:   odr_bits = ICM42670_ACCEL_ODR_25HZ; break;
        case 50:   odr_bits = ICM42670_ACCEL_ODR_50HZ; break;
        case 100:  odr_bits = ICM42670_ACCEL_ODR_100HZ; break;
        case 200:  odr_bits = ICM42670_ACCEL_ODR_200HZ; break;
        case 400:  odr_bits = ICM42670_ACCEL_ODR_400HZ; break;
        case 800:  odr_bits = ICM42670_ACCEL_ODR_800HZ; break;
        case 1600: odr_bits = ICM42670_ACCEL_ODR_1600HZ; break;
        default:   return -2; // invalid ODR
    }

    // Combine into ACCEL_CONFIG0: [7:5] = fsr, [3:0] = odr
    uint8_t accel_config0_val = (fsr_bits << 5) | (odr_bits & 0x0F);
    int rc = icm_i2c_write_byte(ICM42670_ACCEL_CONFIG0_REG, accel_config0_val);
    if (rc != 0) return -3;

    // Set accelerometer to Low Noise (LN) mode via PWR_MGMT0
    // We only touch the lower 2 bits for accel_mode
    rc = icm_i2c_write_byte(ICM42670_PWR_MGMT0_REG, ICM42670_ACCEL_MODE_LN);
    if (rc != 0) return -4;

    return 0; // success
}

int ICM42670_startGyro(uint16_t odr_hz, uint16_t fsr_dps) {
    uint8_t fsr_bits = 0;
    uint8_t odr_bits = 0;
    uint8_t pwr_mgmt0 = 0;

    // Map FSR
    switch (fsr_dps) {
        case 250:  fsr_bits = 0x03; break;
        case 500:  fsr_bits = 0x02; break;
        case 1000: fsr_bits = 0x01; break;
        case 2000: fsr_bits = 0x00; break;
        default:   return -1;
    }

    // Map ODR
    switch (odr_hz) {
        case 25:   odr_bits = 0x0B; break;
        case 50:   odr_bits = 0x0A; break;
        case 100:  odr_bits = 0x09; break;
        case 200:  odr_bits = 0x08; break;
        case 400:  odr_bits = 0x07; break;
        case 800:  odr_bits = 0x06; break;
        case 1600: odr_bits = 0x05; break;
        default:   return -2;
    }

    // Write GYRO_CONFIG0
    uint8_t gyro_config0_val = (fsr_bits << 5) | (odr_bits & 0x0F);
    if (icm_i2c_write_byte(0x20, gyro_config0_val) != 0) return -3;

    // Read-modify-write PWR_MGMT0 to set Gyro mode to LN (bits 3:2 = 11)
    if (i2c_read_blocking(i2c_default, ICM42670_I2C_ADDRESS, (uint8_t[]){0x1F}, 1, true) != 1)
        return -4;
    if (i2c_read_blocking(i2c_default, ICM42670_I2C_ADDRESS, &pwr_mgmt0, 1, false) != 1)
        return -5;

    pwr_mgmt0 &= ~(0b11 << 2);          // Clear bits 3:2
    pwr_mgmt0 |= (0b11 << 2);           // Set to LN
    if (icm_i2c_write_byte(0x1F, pwr_mgmt0) != 0) return -6;

    return 0;
}

int ICM42670_enable_accel_gyro_ln_mode() {
    return icm_i2c_write_byte(0x1F, 0x0F); // bits 3:2 = gyro LN, bits 1:0 = accel LN
}

int ICM42670_read_sensor_data(int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz,int16_t *t) {
        
        uint8_t raw[14]; // 14 bytes total from TEMP to GYRO Z

        int rc = icm_i2c_read_bytes(ICM42670_SENSOR_DATA_START_REG, raw, sizeof(raw));
        if (rc != 0) return rc;

        // Convert to signed 16-bit integers (big-endian)
        *t  = (int16_t)((raw[0] << 8) | raw[1]);
        
        *ax = ((int16_t)raw[2] << 8) | raw[3];
        *ay = ((int16_t)raw[4] << 8) | raw[5];
        *az = ((int16_t)raw[6] << 8) | raw[7];

        *gx = (int16_t)((raw[8] << 8) | raw[9]);
        *gy = (int16_t)((raw[10] << 8) | raw[11]);
        *gz = (int16_t)((raw[12] << 8) | raw[13]);

        return 0; // success
}

