/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

 //NOTE: THIS PROGRAM ONLY WORKS WITH PICOW
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 1000
#endif

// Perform initialisation
// Should use the WiFi driver to control the LED. 
int pico_led_init(void) {
    return cyw43_arch_init();
}

// Turn the led on or off
void pico_set_led(bool led_on) {
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
}

int main() {
    pico_led_init();
    stdio_init_all();
    printf("Initiating\n");
   
    while (true) {
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
        printf("Tick\n");
    }
}
