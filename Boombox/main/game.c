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
#include "esp_random.h"

static const char *TAG = "GAME";

/* ========== MCP23017 Setup ========== */
#define MCP23017_ADDR       0x20
#define I2C_MASTER_FREQ_HZ  (50 * 1000)

/* MCP23017 Registers */
#define IODIRA  0x00
#define IODIRB  0x01
#define GPIOA   0x12
#define GPIOB   0x13
#define GPPUA   0x0C
#define GPPUB   0x0D

/* Port A pins: 3 mistake LEDs + 1 button (existing) */
#define LEDS_MASK_A   0x07   // GPA0, GPA1, GPA2
#define BUTTON_PIN_A  0x08   // GPA3 (active-low)

/* Port B pins: "button game"
 *
 * Wiring assumed:
 *  White:  LED = B0, button = B4
 *  Yellow: LED = B1, button = B5
 *  Green:  LED = B2, button = B6
 *  Red:    LED = B3, button = B7
 *
 * B0–B3: outputs to LED (via resistor) -> LED other pin to GND
 * B4–B7: inputs, one side of switch; other side of switch -> GND
 * Pull-ups ON -> released=1, pressed=0 (active-low).
 */
#define BUTTON_LED_MASK   0x0F   // B0..B3 = button LEDs
#define BUTTON_INPUT_MASK 0xF0   // B4..B7 = button inputs

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

static volatile uint8_t s_led_count = 0;   // still used for mistake LEDs (Port A)

/* ========== "button game" state ========== */

#define BUTTON_GAME_MAX_STEPS   16
#define BUTTON_GAME_WIN_ROUNDS  6   // complete 6 rounds to win

static bool button_mistake = false;


typedef struct {
    uint8_t  seq[BUTTON_GAME_MAX_STEPS];
    int      round;           // current round length (1..BUTTON_GAME_WIN_ROUNDS), 0 = not started
    int      input_index;     // index inside current round
    uint8_t  last_buttons;    // for edge detection
    bool     started;         // becomes true after first button press (game actually starts)
    bool     pattern_showing; // true while pattern is being shown (ignore input, fixed LEDs)
} button_game_t;

static button_game_t s_button_game;


/* ========== MCP23017 low-level ========== */

static esp_err_t mcp_write_reg(uint8_t reg, uint8_t val)
{
    if (!s_mcp_dev) return ESP_FAIL;
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_mcp_dev, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t mcp_read_reg(uint8_t reg, uint8_t *val)
{
    if (!s_mcp_dev) return ESP_FAIL;
    return i2c_master_transmit_receive(s_mcp_dev, &reg, 1, val, 1, pdMS_TO_TICKS(100));
}

static esp_err_t mcp_init_device(void)
{
    esp_err_t ret;

    /* Port A: GPA0-2 outputs (mistake LEDs), GPA3-7 inputs */
    ret = mcp_write_reg(IODIRA, 0xF8); // 1111 1000
    if (ret != ESP_OK) return ret;

    /* Pull-up on GPA3 (button) */
    ret = mcp_write_reg(GPPUA, BUTTON_PIN_A);
    if (ret != ESP_OK) return ret;

    /* All Port A outputs off */
    ret = mcp_write_reg(GPIOA, 0x00);
    if (ret != ESP_OK) return ret;

    /* Port B: B0-3 outputs (button LEDs), B4-7 inputs (buttons) */
    ret = mcp_write_reg(IODIRB, 0xF0); // 1111 0000
    if (ret != ESP_OK) return ret;

    /* Pull-ups on button inputs B4-7 */
    ret = mcp_write_reg(GPPUB, 0xF0);
    if (ret != ESP_OK) return ret;

    /* All Port B outputs off (LEDs off) */
    ret = mcp_write_reg(GPIOB, 0x00);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MCP23017 initialized");
    return ESP_OK;
}

/* ========== 7-seg / buzzer helpers ========== */

static void shiftOut(int dataPin, int clockPin, int bitOrder, uint8_t val)
{
    for (int i = 0; i < 8; i++) {
        int bit;
        if (bitOrder == 1) {
            bit = !!(val & (1 << (7 - i))); // MSBFIRST
        } else {
            bit = !!(val & (1 << i));
        }
        gpio_set_level(dataPin, bit);
        gpio_set_level(clockPin, 1);
        ets_delay_us(1);
        gpio_set_level(clockPin, 0);
    }
}

static void showDigit(int pos, int num, bool withDot)
{
    gpio_set_level(LATCH_PIN, 0);
    uint8_t seg = s_segment_map[num];
    if (withDot) seg |= DP;

    shiftOut(DATA_PIN, CLOCK_PIN, 1, s_digit_map[pos]);
    shiftOut(DATA_PIN, CLOCK_PIN, 1, seg);

    gpio_set_level(LATCH_PIN, 1);
    ets_delay_us(1000);
}

static void displayTime7seg(void)
{
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

static void buzzer_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void buzzer_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void beepBuzzer(int times)
{
    for (int i = 0; i < times; i++) {
        gpio_set_level(LED_PIN, 1);
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_PIN, 0);
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

static void onMinutePassed(void)
{
    ESP_LOGI(TAG, "Minute passed! Timer=%u", s_timerSeconds);
    beepBuzzer(3);
}

/* ========== "button game" helpers ========== */

/* LEDs active-high: mask bit = 1 -> LED ON */
static void button_game_set_leds(uint8_t mask)
{
    /* Only lower 4 bits are LEDs; upper bits are inputs, writing there is harmless. */
    mcp_write_reg(GPIOB, (uint8_t)(mask & BUTTON_LED_MASK));
}

/* Pattern step: LED only, no sound. Keep it visible a bit longer. */
static void button_game_flash_step(int idx)
{
    if (idx < 0 || idx > 3) return;

    uint8_t mask = (uint8_t)(1u << idx);

    /* Pattern step: LED only, no sound. Keep it visible a bit longer. */
    button_game_set_leds(mask);
    vTaskDelay(pdMS_TO_TICKS(400));
    button_game_set_leds(0);
    vTaskDelay(pdMS_TO_TICKS(300));
}

static void button_game_show_pattern(int length)
{
    if (length <= 0) return;
    if (length > BUTTON_GAME_MAX_STEPS) length = BUTTON_GAME_MAX_STEPS;

    s_button_game.pattern_showing = true;
    button_game_set_leds(0);
    vTaskDelay(pdMS_TO_TICKS(250));

    for (int i = 0; i < length; ++i) {
        int idx = (int)s_button_game.seq[i];
        button_game_flash_step(idx);
    }

    /* After showing pattern, leave LEDs off; input phase will light per press */
    button_game_set_leds(0);
    s_button_game.pattern_showing = false;
}

/* Initialize new "button game" */
static void button_game_reset(void)
{
    /* Clear state; actual sequence is created on first button press. */
    for (int i = 0; i < BUTTON_GAME_MAX_STEPS; ++i) {
        s_button_game.seq[i] = 0;
    }

    s_button_game.round           = 0;     // 0 => waiting for first press
    s_button_game.input_index     = 0;
    s_button_game.last_buttons    = 0;
    s_button_game.started         = false;
    s_button_game.pattern_showing = false;

    s_game_won = false;

    /* Ensure all button LEDs are OFF. */
    button_game_set_leds(0);
}

/* Update button game: called from game_update() */
static void button_game_update(void)
{
    if (!s_game_running)        return;
    if (s_game_won)             return;
    if (!s_mcp_dev)             return;

    uint8_t bval = 0;
    if (mcp_read_reg(GPIOB, &bval) != ESP_OK) {
        return;
    }

    /* Buttons on B4-B7, active-low -> map to bits 0..3 (1 = pressed) */
    uint8_t raw_buttons = (uint8_t)((~bval >> 4) & 0x0F);

    /* Do not light anything before the game actually starts */
    if (!s_button_game.started && !s_button_game.pattern_showing) {
        button_game_set_leds(0);
    } else if (!s_button_game.pattern_showing) {
        button_game_set_leds(raw_buttons);
    }


    /* Edge detection: consider newly pressed buttons only */
    uint8_t new_presses = (uint8_t)(raw_buttons & ~s_button_game.last_buttons);
    s_button_game.last_buttons = raw_buttons;

    /* Ignore any presses while we are showing the pattern */
    if (s_button_game.pattern_showing) {
        return;
    }

    /* Game not started yet: wait for the first button press to start.
     * First press only starts the game and shows the first pattern,
     * it is NOT counted as a guess. */
    if (!s_button_game.started) {
        if (new_presses == 0) {
            return;
        }

        for (int i = 0; i < BUTTON_GAME_MAX_STEPS; ++i) {
            s_button_game.seq[i] = (uint8_t)(esp_random() % 4); // values 0..3
        }

        s_button_game.round       = 1;
        s_button_game.input_index = 0;
        s_button_game.started     = true;

        button_game_show_pattern(s_button_game.round);
        return;
    }

    /* From here on the game is running and pattern is not being shown. */
    if (new_presses == 0) return;

    /* Map to a single button index 0..3 (white/yellow/green/red) */
    int pressed = -1;
    if (new_presses & 0x1)      pressed = 0; // white
    else if (new_presses & 0x2) pressed = 1; // yellow
    else if (new_presses & 0x4) pressed = 2; // green
    else if (new_presses & 0x8) pressed = 3; // red

    if (pressed < 0) return;

    int expected = (int)s_button_game.seq[s_button_game.input_index];

    if (pressed == expected) {
        /* Correct step */
        s_button_game.input_index++;

        if (s_button_game.input_index >= s_button_game.round) {
            /* Round completed */
            game_beep_win();

            s_button_game.round++;
            s_button_game.input_index = 0;

            if (s_button_game.round > BUTTON_GAME_WIN_ROUNDS) {
                /* All rounds done -> module win */
                s_game_won = true;
                button_game_set_leds(0);
                ESP_LOGI(TAG, "Button game completed (rounds=%d)", BUTTON_GAME_WIN_ROUNDS);
                return;
            }

            /* Show extended pattern for next round */
            button_game_show_pattern(s_button_game.round);
        }
    } else {
        /* Mistake: error beep, then repeat same pattern length */
        ESP_LOGI(TAG, "Button game mistake: pressed=%d, expected=%d",
                 pressed, expected);
        button_mistake = true;         
        game_beep_mistake();
        s_button_game.input_index = 0;

        button_game_show_pattern(s_button_game.round);
    }
}

/* ========== Public API ========== */

void game_init(i2c_master_bus_handle_t mcp_bus)
{
    s_mcp_bus = mcp_bus;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MCP23017_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_mcp_bus, &dev_cfg, &s_mcp_dev));
    ESP_ERROR_CHECK(mcp_init_device());

    /* 7-seg + LED pins */
    gpio_config_t io_conf_out = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LATCH_PIN) |
                        (1ULL << DATA_PIN)  |
                        (1ULL << CLOCK_PIN) |
                        (1ULL << LED_PIN),
    };
    gpio_config(&io_conf_out);

    /* Buzzer PWM */
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

    button_game_reset();
}

void game_start(int total_seconds)
{
    s_timerSeconds = total_seconds;
    s_lastUpdate   = esp_timer_get_time();
    s_lastMinutes  = s_timerSeconds / 60;

    s_game_running = true;
    s_game_won     = false;
    s_time_up      = false;
    s_led_count    = 0;

    if (s_mcp_dev != NULL) {
        /* Clear Port A and B outputs (mistake LEDs + button LEDs) */
        mcp_write_reg(GPIOA, 0x00);
        mcp_write_reg(GPIOB, 0x00);
        mcp_write_reg(GPIOB, 0x00);
    }

    /* Reset button game state; game actually starts on first button press */
    button_game_reset();
}

void game_update(void)
{
    if (!s_game_running) return;

    /* timer */
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

    /* 7-seg display */
    displayTime7seg();

    /* Button Game on MCP23017 Port B */
    button_game_update();
}

bool game_is_won(void)
{
    return s_game_won;
}

bool game_is_time_up(void)
{
    return s_time_up;
}

void game_stop(void)
{
    s_game_running = false;
    buzzer_off();

    if (s_mcp_dev != NULL) {
        mcp_write_reg(GPIOA, 0x00);   // Port A LEDs off
        mcp_write_reg(GPIOB, 0x00);   // button LEDs off
    }
}

void game_update_mistake_leds(int mistakes)
{
    if (s_mcp_dev == NULL) return;

    uint8_t val = 0;

    if (mistakes >= 1) val |= (1 << 0);
    if (mistakes >= 2) val |= (1 << 1);
    if (mistakes >= 3) val |= (1 << 2);

    mcp_write_reg(GPIOA, val);
}

/* ========== beep API ========== */

void game_beep_mistake(void)
{
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(120));
    buzzer_off();
}

void game_beep_win(void)
{
    /* two short beeps */
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(120));
    buzzer_off();
    vTaskDelay(pdMS_TO_TICKS(120));
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(200));
    buzzer_off();
}

void game_beep_fail(void)
{
    /* one long beep */
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(400));
    buzzer_off();
}

bool button_mistake_done(void)
{
    if (button_mistake) {
        button_mistake = false;  // one-shot
        return true;
    }
    return false;
}

void game_enable_buttons(void)
{
    button_game_reset();   // fresh sequence, waits for first valid press
}
