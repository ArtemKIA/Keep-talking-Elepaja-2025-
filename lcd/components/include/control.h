#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/gpio.h"

static const char *TAG = "MAIN";


/* ------------------ CONFIG ------------------ */
#define Y_CHANNEL ADC1_CHANNEL_4  // GPIO34 on ESP32 || GPIO8 on ESP32-S3
#define BUTTON    GPIO_NUM_9      // ESP32-S3 pin 46

#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_11
#define DEADZONE  300
#define SCROLL    2000

typedef enum {
    SCROLL_UP,
    SCROLL_DOWN,
    NO_SCROLL,
    PRESS
} joystick_event_t;


/* ------------------ Joystick ------------------ */
void joystick_init(void) {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(Y_CHANNEL, ADC_ATTEN);

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button_config);
    ESP_LOGI(TAG, "Joystick initialized");
}

joystick_event_t joystick_read_event(void) {
    static joystick_event_t last_event = NO_SCROLL;
    joystick_event_t new_event = NO_SCROLL;

    int16_t y_value = adc1_get_raw(Y_CHANNEL);
    int button_state = gpio_get_level(BUTTON);

    // Button debounce
    if (button_state == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (gpio_get_level(BUTTON) == 0) new_event = PRESS;
    } else {
        int16_t y_diff = y_value - 2048;
        if (abs(y_diff) > SCROLL) {
            if (y_diff < 0) new_event = SCROLL_UP;
            else new_event = SCROLL_DOWN;
        }
    }

    // Trigger only on edge
    if (new_event != NO_SCROLL && new_event != last_event) {
        last_event = new_event;
        return new_event;
    }

    if (new_event == NO_SCROLL) last_event = NO_SCROLL;
    return NO_SCROLL;
}


#endif