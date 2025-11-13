#include "game.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

static const char *TAG = "GAME";

/* ========== MCP23017 Setup ========== */
#define MCP23017_ADDR 0x20
#define I2C_MASTER_FREQ_HZ (50*1000)

/* MCP23017 Registers */
#define IODIRA 0x00
#define GPIOA  0x12
#define GPPUA  0x0C

/* MCP23017 Pins */
#define LEDS_MASK  0x07   // GPA0, GPA1, GPA2
#define BUTTON_PIN 0x08   // GPA3

/* MCP23017 Pins */
#define LEDS_MASK  0x07   // GPA0, GPA1, GPA2
#define BUTTON_PIN 0x08   // GPA3

/* ========== 7-seg / shift-reg pins ========== */
#define LATCH_PIN  GPIO_NUM_9
#define DATA_PIN   GPIO_NUM_18
#define CLOCK_PIN  GPIO_NUM_10

/* Extra outputs */
#define LED_PIN    GPIO_NUM_45
#define BUZZER_PIN GPIO_NUM_4

/* 7-seg maps */
static const uint8_t s_digit_map[4] = {
    0b00001110, 0b00001101, 0b00001011, 0b00000111
};
static const uint8_t s_segment_map[10] = {
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
#define DP 0b10000000

/* ========== internal state ========== */

static i2c_master_bus_handle_t  s_mcp_bus = NULL;
static i2c_master_dev_handle_t  s_mcp_dev = NULL;

static unsigned int s_timerSeconds = 0;
static int64_t      s_lastUpdate   = 0;
static unsigned int s_lastMinutes  = 0;

static volatile bool s_game_running = false;
static volatile bool s_game_won     = false;
static volatile bool s_time_up      = false;

static volatile uint8_t s_led_count = 0;

/* ========== MCP23017 low-level ========== */

static esp_err_t mcp_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_mcp_dev, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t mcp_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(s_mcp_dev, &reg, 1, val, 1, pdMS_TO_TICKS(100));
}

static esp_err_t mcp_init_device(void) {
    esp_err_t ret;
    ret = mcp_write_reg(IODIRA, 0xF8); // GPA0-2 out, GPA3-7 in
    if (ret != ESP_OK) return ret;
    ret = mcp_write_reg(GPPUA, 0x08); // pull-up on GPA3
    if (ret != ESP_OK) return ret;
    ret = mcp_write_reg(GPIOA, 0x00);// all LEDs off
    ESP_LOGI(TAG, "MCP23017 initialized");
    return ret;
}


/* ========== 7-seg / buzzer helpers ========== */

static void shiftOut(int dataPin, int clockPin, int bitOrder, uint8_t val) {
    for (int i = 0; i < 8; i++) {
        int bit;
        if (bitOrder == 1)
            bit = !!(val & (1 << (7 - i))); // MSBFIRST
        else
            bit = !!(val & (1 << i));
        gpio_set_level(dataPin, bit);
        gpio_set_level(clockPin, 1);
        ets_delay_us(1);
        gpio_set_level(clockPin, 0);
    }
}

static void showDigit(int pos, int num, bool withDot) {
    gpio_set_level(LATCH_PIN, 0);
    uint8_t seg = s_segment_map[num];
    if (withDot) seg |= DP;
    shiftOut(DATA_PIN, CLOCK_PIN, 1, s_digit_map[pos]);
    shiftOut(DATA_PIN, CLOCK_PIN, 1, seg);
    gpio_set_level(LATCH_PIN, 1);
    ets_delay_us(1000);
}

static void displayTime7seg(void) {
    int minutes = s_timerSeconds / 60;
    int seconds = s_timerSeconds % 60;

    int d1 = minutes / 10;
    int d2 = minutes % 10;
    int d3 = seconds / 10;
    int d4 = seconds % 10;

    showDigit(0, d1, false);
    showDigit(1, d2, true);
    showDigit(2, d3, false);
    showDigit(3, d4, false);
}

static void buzzer_on(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void buzzer_off(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void beepBuzzer(int times) {
    for (int i = 0; i < times; i++) {
        gpio_set_level(LED_PIN, 1);
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_PIN, 0);
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

static void onMinutePassed(void) {
    ESP_LOGI(TAG, "Minute passed! Timer=%u", s_timerSeconds);
    beepBuzzer(3);
}

/* ========== Public API ========== */

void game_init(i2c_master_bus_handle_t mcp_bus) {
    s_mcp_bus = mcp_bus;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MCP23017_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_mcp_bus, &dev_cfg, &s_mcp_dev));
    ESP_ERROR_CHECK(mcp_init_device());

    // 7-seg + LED pins
    gpio_config_t io_conf_out = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LATCH_PIN) |
                        (1ULL << DATA_PIN)  |
                        (1ULL << CLOCK_PIN) |
                        (1ULL << LED_PIN),
    };
    gpio_config(&io_conf_out);

    // Buzzer PWM
    ledc_timer_config_t buzzer_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz         = 2000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&buzzer_timer);

    ledc_channel_config_t buzzer_channel = {
        .gpio_num   = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&buzzer_channel);

    s_game_running = false;
    s_game_won     = false;
    s_time_up      = false;
    s_led_count    = 0;
}

void game_start(int total_seconds) {
    s_timerSeconds = total_seconds;
    s_lastUpdate   = esp_timer_get_time();
    s_lastMinutes  = s_timerSeconds / 60;

    s_game_running = true;
    s_game_won     = false;
    s_time_up      = false;
    s_led_count    = 0;

    mcp_write_reg(GPIOA, 0x00); // LEDs off
}

void game_update(void) {
    if (!s_game_running) return;

    // timer
    int64_t now = esp_timer_get_time();
    if ((now - s_lastUpdate) >= 1000000 && s_timerSeconds > 0) {
        s_timerSeconds--;
        s_lastUpdate = now;

        unsigned int currentMinutes = s_timerSeconds / 60;
        if (currentMinutes != s_lastMinutes) {
            s_lastMinutes = currentMinutes;
            onMinutePassed();
        }
        if (s_timerSeconds == 0) {
            s_time_up = true;
        }
    }

    // 7-seg
    displayTime7seg();

    // MCP23017 mini-game
    uint8_t read_val;
    static uint8_t last_button_state = 1;
    uint8_t current_button_state;

    if (mcp_read_reg(GPIOA, &read_val) == ESP_OK) {
        current_button_state = (read_val & BUTTON_PIN) ? 0 : 1; // active low

        if (current_button_state == 1 && last_button_state == 0) {
            s_led_count++;
            if (s_led_count > 7) s_led_count = 0;

            mcp_write_reg(GPIOA, (read_val & ~LEDS_MASK) | (s_led_count & LEDS_MASK));
            if (s_led_count == 7) {
                s_game_won = true;
                ESP_LOGI(TAG, "MCP game: LEDs=111, win");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        last_button_state = current_button_state;
    }
}

bool game_is_won(void) {
    return s_game_won;
}

bool game_is_time_up(void) {
    return s_time_up;
}

void game_stop(void) {
    s_game_running = false;
    buzzer_off();
}