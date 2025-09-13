
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <queue.h>
#include <task.h>

#include <shellSDK/SDK.h>
#include <pico/binary_info.h>

// Flag to indicate button press
volatile bool sw1_pressed = false;
volatile bool sw2_pressed = false;

void led_simple_task(void *pwParameters);
void sw1_task(void *pvParameters);
void sw2_task(void *pvParameters);
void led_task(void *pvParameters);
void rgb_task(void *pvParameters);
void buzzer_task(void *pvParameters);
void light_sensor_task(void *pvParameters);
void ths_task(void *pvParameters);
void imu_task(void *pvParameters);


// void display_task(void *pvParameters);
// void mic_task(void *pvParameters);


// // Callback function when PDM samples are ready
// void on_mic_data_ready(){
//     // Read the samples into the sample_buffer
//     samples_read =pdm_microphone_read(sample_buffer, 256);

// }




void sw2_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        sw2_pressed = gpio_get(SW2_PIN);
        vTaskDelay(10);
    }
}

void rgb_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        rgb_led_write(20, 30, 255);
        vTaskDelay(500);
        rgb_led_write(255, 30, 10);
        vTaskDelay(500);
        rgb_led_write(50, 255, 10);
        vTaskDelay(500);
    }
}

void buzzer_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        if (sw2_pressed) {
            buzzer_play_tone(440, 500);
        }
        vTaskDelay(10);
    }
}

void light_sensor_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        uint16_t light = veml6030_read_light();
        printf("Light level: %d\n", light);
        vTaskDelay(1000);
    }
}

void ths_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        float temp = hdc2021_read_temperature();
        float humid = hdc2021_read_humidity();
        printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temp, humid);
        vTaskDelay(1000);
    }
}


void imu_task(void *pvParameters) {
    (void)pvParameters;
    
    int16_t ax, ay, az, gx, gy, gz, t;

    while (1)
    {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            float temp_c = (float)t / 128.0f;
            printf("Accel: X=%d, Y=%d, Z=%d | Gyro: X=%d, Y=%d, Z=%d | Temp: %.2f°C\n", ax, ay, az, gx, gy, gz, temp_c);

        } else {
            printf("Failed to read imu data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(60));
    }

}

// void display_task(void *pvParameters) {

//     (void)pvParameters;

//     while (1) {
//         clear_display();
//         // write_text("Hello");
//         // draw_circle(1, 1, 20);
//         // draw_line(0, 0, 127, 0);
//         draw_square(30,10,20,20);
//         draw_square(70,10,20,20);
//         vTaskDelay(1000);
//         clear_display();
//         draw_line(30, 20, 50, 20);
//         draw_line(70, 20, 90, 20);
//         vTaskDelay(300);

//     }
// }

// void mic_task(void *pvParameters) {
//     (void)pvParameters;

//     while (1) {
//        if (samples_read !=0){

//         // store and clear the samples read from the callback
//         int sample_count = samples_read;
//         samples_read = 0;

//         // loop through any new collected samples
//         for (int i = 0; i < sample_count; i++) {
//             printf("%b\n", sample_buffer[i]);
//         }
//        }
//     }
// }

//





/**
 * @brief FreeRTOS task for testing the red LED.
 *
 * This task toggles the state of the red LED once at startup,
 * then enters an infinite loop where it simply delays by 1000 ticks
 * on each iteration. The task yields to the scheduler, allowing other
 * tasks to run, but does not continue toggling the LED after the first change.
 *
 * @param[in] pvParameters
 *        Pointer to task parameters. Not used in this implementation,
 *        should be passed as NULL when creating the task.
 *
 * @note This implementation is useful to verify task creation and scheduling,
 *       but it does not produce a blinking LED.
 * @see toggle_red_led()
 * @see vTaskDelay()
 */
void led_simple_task(void *pvParameters) {
    toggle_red_led();
    while(1){
        vTaskDelay(1000);
    };
}

/**
 * @brief FreeRTOS task for monitoring switch SW1 state.
 *
 * This task continuously reads the digital input connected to SW1
 * and stores its state in the global variable @ref sw1_pressed.
 *
 * @param[in] pvParameters
 *        Pointer to task parameters. Not used in this implementation,
 *        should be passed as NULL when creating the task.
 * @see gpio_get()
 * @see vTaskDelay()
 */
void sw1_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        
        sw1_pressed = gpio_get(SW1_PIN);
        vTaskDelay(10);
    }
}

/**
 * @brief FreeRTOS task for controlling the red LED.
 *
 * This task continuously sets the red LED output level based on the
 * global variable @ref sw1_pressed, which reflects the current state
 * of switch SW1. If the switch is pressed, the LED is turned on;
 * otherwise, it is turned off.
 * @param[in] pvParameters
 *        Pointer to task parameters. Not used in this implementation,
 *        should be passed as NULL when creating the task.
 *
 * @warning This task depends on @ref sw1_task to update @ref sw1_pressed.
 *          Ensure that @ref sw1_task is running concurrently; otherwise,
 *          the LED state will not change.
 *
 * @see gpio_put()
 * @see vTaskDelay()
 */
void led_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        gpio_put(RED_LED_PIN, sw1_pressed);
        /*if (!sw1_pressed){
            printf("Button pressed");
        }*/
        vTaskDelay(100);
    }
}
    /*============================
        MICROPHONE CONFIGURATION
     =============================*/
    //Internal sample buffer for sound samples
    int16_t sample_buffer[MEMS_BUFFER_SIZE];
    int16_t temp_sample_buffer[MEMS_BUFFER_SIZE];//use to have two different buffers.
    volatile int samples_read = 0;    

    void on_sound_buffer_ready(){
        // callback from library when all the samples in the library
        // internal sample buffer are ready for reading 
        samples_read = get_microphone_samples(sample_buffer, MEMS_BUFFER_SIZE);
    }

int main() {
    stdio_init_all();
    sleep_ms(2000); //Wait to see the output.
    init_shell();
    printf("Start tests\n");
    
    // Initialize LED
    init_red_led();
    printf("Initializing red led\r\n");
    
    //Testing RED LED
    // printf("Testing red led should be on\r\n");
   


    // Test SW1 
    //init_sw1();
    //printf("Initializing switch 1\r\n");

    //Test SW2
    //init_sw2();
    //printf("Initializing switch 2\r\n");

    //Test RGB
    //init_rgb_led();
    //printf("Initializing RGB LED\r\n");


   // Initialize Buzzer
   //init_buzzer();
   //printf("Initializing the buzzer\n");

   
    // Initialize I2C
    // i2c_init_default(DEFAULT_I2C_SDA_PIN, DEFAULT_I2C_SCL_PIN);
    // printf("Initializing the i2c\n");

    //Initialize Light Sesnsor VEML6030
    //veml6030_init();
    //printf("Initializing the light sensor\n");

    //Initialize the Temp and Humidity Sensor
    //hdc2021_init();
    //printf("Initializing the temp/humidity sensor\n");

    //Initialize the display
    //init_display();
    //printf("Initializating display\n");

    //Initialize the microphone
    // do not buffer the output for the microphone. 
    setvbuf(stdout, NULL, _IONBF, 0);
    int is_mic_init = init_pdm_microphone();
    if (is_mic_init < 0){
        printf("PDM microphone initialization failed!\n");
        sleep_ms(100);
    }
    else 
        printf("Initializing the microphone");


    while(true){
        set_red_led_status(false);
        //printf("SW1 state: %d\n", gpio_get(SW1_PIN));
        //printf("SW2 state: %d\n", gpio_get(SW2_PIN));
        //buzzer_play_tone(440, 500);
        /*rgb_led_write(255,0,0);
        sleep_ms(1000);
        rgb_led_write(0,255,0);
        sleep_ms(1000);
        rgb_led_write(0,0,255);*/
        //uint16_t reg = _veml6030_read_register(VEML6030_CONFIG_REG);
        //printf("Register: 0x%04X\n",(unsigned int)reg);
        //uint32_t light = veml6030_read_light();
        //printf("Light level: %u\n", light);
        //float temp = hdc2021_read_temperature();
        //float humid = hdc2021_read_humidity();
        //printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temp, humid);
        // clear_display();
        // write_text("Hello");
        // draw_circle(25, 25, 20, true);
        //draw_line(0, 0, 127, 0);
        //draw_square(30,10,20,20, true);
        // draw_square(70,10,20,20, false);
        // write_text_xy(20,40, "Bye");

        //MICROPHONE: 
        // set callback that is called when all the samples in the library
        // internal sample buffer are ready for reading
     
        if (is_mic_init >=0) {
            pdm_microphone_set_callback(on_sound_buffer_ready);
            if (init_microphone_sampling()<0){
                printf("Cannot start sampling the microphone\n");
                sleep_ms(500);
                continue;
            }
            // Wait till usb is ready and after that inform other end with READY.
            while (!stdio_usb_connected()) 
                sleep_ms(100);
            //We are going to send 5 seconds. Each sample is two bytes and sampling rate 8Khz. 
            uint32_t target_bytes = MEMS_SAMPLING_FREQUENCY * 2u * 5u;
            uint32_t sent_bytes = 0;
            puts("READY");
            toggle_red_led();
            while (sent_bytes < target_bytes){
                sleep_ms(150); 
                while (samples_read == 0){
                    sleep_ms(10);
                }
                // store and clear the samples read from the callback
                int sample_count = samples_read;               
                
                // loop through any new collected samples
                // OPTION 1 using fwrite
                // First we create a temporary buffer, so i can send data even if I receive another irq. 
                uint32_t irq = save_and_disable_interrupts();
                memcpy(temp_sample_buffer, sample_buffer, (size_t)sample_count * sizeof(sample_buffer[0]));
                samples_read = 0;// restart the samples read
                restore_interrupts(irq);
                int sample_sent = fwrite(temp_sample_buffer,sizeof(temp_sample_buffer[0]),sample_count,stdout);
                sent_bytes += sizeof(temp_sample_buffer[0]) * sample_sent;
                 
                //stdio_flush();

                //OPTION 2 using putchar
                /*for (int i = 0; i < sample_count; i++) {
                    int16_t s = sample_buffer[i];
                    putchar_raw((int8_t)(s & 0xFF));       // LSB
                    ++sent_bytes;
                    putchar_raw((int8_t)(s >> 8));         // MSB
                    ++sent_bytes;
                }*/
                //stdio_flush();    

                //OPTION 3: using printf. Only for showing in graph (e.g. in Arduino Uno plotter)
                /*for (int i = 0; i < sample_count; i++) {
                    printf("%d\n", sample_buffer[i]);
                    sent_bytes += sizeof(sample_buffer[0]);
                }
                stdio_flush();*/

            /*RECEIVE IN SERIAL USB (CDC) using the following code: 
            # put the port in raw mode (important: no line processing)
            stty -F /dev/ttyACM0 raw -echo -echoe -echok
            # read until READY
            grep -m1 -a "READY" < /dev/ttyACM0 >/dev/null
            # read exactly 5 s of audio: 16000 samples/s * 2 bytes/sample * 5s = 160000 bytes
            head -c 160000 /dev/ttyACM0 > mic_5s_s16le_16k.raw*/
            }
            toggle_red_led();
        }
        sleep_ms(5000);
    }

 


    
    

    //Initialize Temp and Humidity Sesnsor HDC2021
    // hdc2021_init();



    //Initialize IMU
    /*if (ICM42670_init()) {
        printf("ICM-42670P initialized successfully!\n");
        ICM42670_startAccel(100, 16);
        ICM42670_startGyro(100, 250);
        ICM42670_enable_accel_gyro_ln_mode();
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }*/

    // init_pdm_microphone();
    // start_pdm_microphone();
    // pdm_microphone_set_callback(on_mic_data_ready);

    // init_THS();

    // Create tasks
    
    // xTaskCreate(sw2_task, "SW2Task", 256, NULL, 1, NULL);
    // xTaskCreate(rgb_task, "RGBTask", 256, NULL, 1, NULL);
    // xTaskCreate(buzzer_task, "BuzzerTask", 256, NULL, 4, NULL);
    // xTaskCreate(light_sensor_task, "LightSensorTask", 256, NULL, 3, NULL);
    // xTaskCreate(ths_task, "THSTask", 256, NULL, 1, NULL);
    // xTaskCreate(imu_task, "IMUTask", 256, NULL, 1, NULL);
    // xTaskCreate(led_simple_task, "LEDSimpleTask", 64, NULL, 1, NULL);
    // xTaskCreate(sw1_task, "SW1Task", 64, NULL, 1, NULL);
    // xTaskCreate(led_task, "LEDTask", 64, NULL, 2, NULL);
 

    //  xTaskCreate(display_task, "DisplayTask", 256, NULL, 2, NULL);
    // // xTaskCreate(mic_task, "MicTask", 256, NULL, 1, NULL);
    
    // Start the FreeRTOS scheduler
    //vTaskStartScheduler();

    return 0;
}

