
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <pico/stdlib.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"



int main() {
    stdio_init_all();
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(200);



    
    return 0;
}

