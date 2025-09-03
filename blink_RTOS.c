/**
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Pico W devices use a GPIO on the Wi-Fi chip for the LED
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 1000
#endif

// Init LED backend
static int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    return cyw43_arch_init(); // for Pico W
#else
    return PICO_ERROR_GENERIC;
#endif
}

// Set LED on/off
static void pico_set_led(bool on) {
#if defined(PICO_DEFAULT_LED_PIN)
    gpio_put(PICO_DEFAULT_LED_PIN, on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
#endif
}

// ---- FreeRTOS task ----
static void led_task(void *arg) {
    (void)arg;

    // Optional: wait a bit so USB CDC enumerates and early prints are visible
    absolute_time_t until = make_timeout_time_ms(3000);
    while (!stdio_usb_connected() && absolute_time_diff_us(get_absolute_time(), until) > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

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

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    // Create the single blinky/printf task
    xTaskCreate(led_task, "led", 512, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Should never get here
    while (true) { tight_loop_contents(); }
}
