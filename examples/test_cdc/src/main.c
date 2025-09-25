#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "tusb.h"

#define BUFFER_SIZE     20
#define TEMP_MIN        0
#define TEMP_MAX        40
#define LUX_MIN         100
#define LUX_MAX         1500
#define CDC_INTERFACE   1

static int rand_in_range(int min, int max) {
    return min + rand() % (max - min + 1);
}

int main (void) {
    // Initialize TinyUSB first
    tusb_init();

    //Initialize all USB (including stdio as CDC0)
    stdio_init_all();
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }

    sleep_ms(200);

    //Generating random seed
    srand((unsigned)time(NULL));

    printf("=== Dual CDC Example Started ===\n");
    printf("CDC0: Debug output (this interface)\n");
    printf("CDC1: Sending serial data\n");

    char buf[BUFFER_SIZE]; 
    while (1){
        // Process TinyUSB tasks (this must be called regularly)
        tud_task();

        //Generated random numbers
        int temp = rand_in_range(TEMP_MIN, TEMP_MAX);
        int lux  = rand_in_range(LUX_MIN,  LUX_MAX);

        //Send them using 
        snprintf(buf, BUFFER_SIZE,"temp:%d, light:%d\n\0", temp, lux);
        printf("[DEBUG] Generated : %s", buf);

        // Send random values in hexadecimal formal. Transform to str to better understanding, but in real life thye could be just byte data.
        tud_cdc_n_write(CDC_INTERFACE,buf,BUFFER_SIZE);
        sleep_ms (4000);
    }

    
}