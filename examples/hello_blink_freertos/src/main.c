/**
 * SPDX-License-Identifier: BSD-3-Clause
 */
 //NOTE: THIS PROGRAM ONLY WORKS WITH PICOW

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 1000
#endif

// Init LED backend
static int pico_led_init(void) {
    return cyw43_arch_init(); // for Pico W
}

// Set LED on/off
static void pico_set_led(bool on) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

// ---- FreeRTOS task ----
static void led_task(void *arg) {
    (void)arg;

    // Wait until the USB is connected
    // Uncomment following while if there are problems with the USB.
    //while (!stdio_usb_connected()) {
    //    vTaskDelay(pdMS_TO_TICKS(10));
    //}

    for (;;) {
        pico_set_led(true);
        printf("Tack\n");
        vTaskDelay(pdMS_TO_TICKS(LED_DELAY_MS));
        pico_set_led(false);
        vTaskDelay(pdMS_TO_TICKS(LED_DELAY_MS));
    }
}

int main(void) {
    stdio_init_all();

    pico_led_init();
   
    // Create the single blinky/printf task
    xTaskCreate(led_task, "led", 512, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start the scheduler (never returns)
    vTaskStartScheduler();

}
