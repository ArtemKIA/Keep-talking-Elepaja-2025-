#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED1 4
#define LED2 5
#define LED3 6
#define LED4 7

void app_main(void)
{
    // Configure the GPIOs as output
    gpio_reset_pin(LED1);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED2);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED3);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED4);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);

    gpio_num_t leds[] = {LED1, LED2, LED3, LED4};
    int num_leds = sizeof(leds) / sizeof(leds[0]);

    while (1) {
        for (int i = 0; i < num_leds; i++) {
            // Turn current LED ON
            gpio_set_level(leds[i], 1);
            vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second

            // Turn it OFF before moving to next
            gpio_set_level(leds[i], 0);
        }
    }
}
