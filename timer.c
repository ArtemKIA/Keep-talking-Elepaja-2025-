#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

// ---------------- Pin config ----------------
#define LATCH_PIN 21  // ST_CP
#define DATA_PIN  18  // DS
#define CLOCK_PIN 19  // SH_CP
#define LED_PIN   2   // Onboard LED
#define BUZZER_PIN 4 // Passive buzzer

// ---------------- Maps ----------------
static const uint8_t digit_map[4] = {
    0b00001110,
    0b00001101,
    0b00001011,
    0b00000111
};

static const uint8_t segment_map[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

#define DP 0b10000000  // decimal point bit

// ---------------- Timer variables ----------------
static unsigned int timerSeconds = 5 * 60;
static int64_t lastUpdate = 0;
static unsigned int lastMinutes = 0;

// ---------------- Utility functions ----------------
void shiftOut(int dataPin, int clockPin, int bitOrder, uint8_t val) {
    for (int i = 0; i < 8; i++) {
        int bit;
        if (bitOrder == 1) {
            bit = !!(val & (1 << (7 - i))); // MSBFIRST
        } else {
            bit = !!(val & (1 << i));       // LSBFIRST
        }
        gpio_set_level(dataPin, bit);
        gpio_set_level(clockPin, 1);
        ets_delay_us(1);
        gpio_set_level(clockPin, 0);
    }
}

void setTimer(unsigned int minutes, unsigned int seconds) {
    timerSeconds = minutes * 60 + seconds;
    lastMinutes = timerSeconds / 60;
}

// ---------------- Buzzer control ----------------
void buzzer_on() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128); // 50% duty (out of 255)
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void buzzer_off() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void beepBuzzer(int times) {
    for (int i = 0; i < times; i++) {
        gpio_set_level(LED_PIN, 1);
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(200)); // 200 ms ON
        gpio_set_level(LED_PIN, 0);
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(150)); // 150 ms OFF
    }
}

// ---------------- Timer events ----------------
void onMinutePassed(void) {
    printf("Minute passed! Timer = %u seconds\n", timerSeconds);
    beepBuzzer(3); // 3 short pips
}

void updateTimer() {
    int64_t now = esp_timer_get_time();
    if ((now - lastUpdate) >= 1000000 && timerSeconds > 0) {
        timerSeconds--;
        lastUpdate = now;

        unsigned int currentMinutes = timerSeconds / 60;
        if (currentMinutes != lastMinutes) {
            lastMinutes = currentMinutes;
            onMinutePassed();
        }
    }
}

// ---------------- Display ----------------
void showDigit(int pos, int num, bool withDot) {
    gpio_set_level(LATCH_PIN, 0);

    uint8_t seg = segment_map[num];
    if (withDot) seg |= DP;

    shiftOut(DATA_PIN, CLOCK_PIN, 1, digit_map[pos]);
    shiftOut(DATA_PIN, CLOCK_PIN, 1, seg);

    gpio_set_level(LATCH_PIN, 1);
    ets_delay_us(1000); // ~1 ms
}

void displayTime() {
    int minutes = timerSeconds / 60;
    int seconds = timerSeconds % 60;

    int d1 = minutes / 10;
    int d2 = minutes % 10;
    int d3 = seconds / 10;
    int d4 = seconds % 10;

    showDigit(0, d1, false);
    showDigit(1, d2, true);
    showDigit(2, d3, false);
    showDigit(3, d4, false);
}

// ---------------- Main task ----------------
void app_main(void) {
    // Setup GPIO for shift register + LED
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LATCH_PIN) |
                        (1ULL << DATA_PIN)  |
                        (1ULL << CLOCK_PIN) |
                        (1ULL << LED_PIN)
    };
    gpio_config(&io_conf);

    // Setup buzzer with LEDC PWM
    ledc_timer_config_t buzzer_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 2000,  // 2 kHz beep
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&buzzer_timer);

    ledc_channel_config_t buzzer_channel = {
        .gpio_num       = BUZZER_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // start silent
        .hpoint         = 0
    };
    ledc_channel_config(&buzzer_channel);

    lastUpdate = esp_timer_get_time();
    lastMinutes = timerSeconds / 60;

    while (1) {
        updateTimer();
        displayTime();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
