// ESP-IDF v 5.5.1
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"


// ================== CONFIG ==================
#define SDA_PIN              8          // ESP32-S3 I2C SDA
#define SCL_PIN              9          // ESP32-S3 I2C SCL
#define I2C_SPEED_HZ         50000      // 50 kHz
#define MATRIX_BRIGHT        1          // 0..15

// HT16K33 scan range 0x70..0x77
static uint8_t g_ht16k33_addr = 0x70;

// Joystick: ADC1 channels (ESP32-S3) — GPIO4 -> CH3, GPIO5 -> CH4
#define JOY_VRX_CH           ADC_CHANNEL_3
#define JOY_VRY_CH           ADC_CHANNEL_4
#define JOY_ADC_UNIT         ADC_UNIT_1
#define JOY_SW_PIN           6          // button (active low, pull-up)

// Movement tuning
#define MOVE_COOLDOWN_MS    200
#define DEADZONE_MIN        350
#define DEADZONE_MAX        1400

// Blink + loop timing
#define BLINK_MS            500
#define FRAME_DELAY_MS      15

// ================== I2C (new driver) ==================
static i2c_master_bus_handle_t i2c_bus   = NULL;
static i2c_master_dev_handle_t ht16k33   = NULL;

static void i2c_init_bus(void) {
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));
}

static bool ht16k33_probe_and_init(void) {
    for (uint8_t addr = 0x70; addr <= 0x77; addr++) {
        if (i2c_master_probe(i2c_bus, addr, 50) == ESP_OK) {
            g_ht16k33_addr = addr;

            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address  = g_ht16k33_addr,
                .scl_speed_hz    = I2C_SPEED_HZ,
            };
            ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &ht16k33));

            uint8_t cmd;
            cmd = 0x21;  ESP_ERROR_CHECK(i2c_master_transmit(ht16k33, &cmd, 1, -1));                   // oscillator on
            cmd = 0x81;  ESP_ERROR_CHECK(i2c_master_transmit(ht16k33, &cmd, 1, -1));                   // display on, blink off
            cmd = (uint8_t)(0xE0 | (MATRIX_BRIGHT & 0x0F)); ESP_ERROR_CHECK(i2c_master_transmit(ht16k33, &cmd, 1, -1)); // brightness
            return true;
        }
    }
    return false;
}

static void ht16k33_write_rows(uint8_t rows[8]) {
    uint8_t buf[1 + 16];
    buf[0] = 0x00; // start address
    for (int r = 0; r < 8; r++) {
        buf[1 + r*2]     = rows[r]; // even addr: row bits
        buf[1 + r*2 + 1] = 0x00;    // odd addr: unused
    }
    ESP_ERROR_CHECK(i2c_master_transmit(ht16k33, buf, sizeof(buf), -1));
}

static void ht16k33_clear(void) {
    uint8_t rows[8] = {0};
    ht16k33_write_rows(rows);
}

// ================== MAZE DATA ==================
static uint8_t maze[8][8] = {
  {1,1,1,1,1,1,1,1},
  {1,0,0,0,0,0,0,1},
  {1,1,1,1,0,1,0,1},
  {1,0,0,0,0,1,0,1},
  {1,1,1,0,1,1,0,1},
  {1,0,0,0,0,1,0,1},
  {1,0,1,1,0,0,0,1},
  {1,0,1,1,1,1,1,1}
};

static uint8_t mazeShow[8][8] = {
  {0,1,0,0,0,0,0,0},
  {1,0,0,0,0,0,0,0},
  {0,1,0,1,0,1,0,0},
  {0,0,0,0,0,1,0,0},
  {0,1,0,0,0,0,0,0},
  {0,0,0,0,0,1,0,0},
  {0,0,1,0,0,0,0,0},
  {0,0,0,0,0,0,0,0}
};

// ================== GAME STATE ==================
static int playerX = 1;
static int playerY = 1;
static int goalX   = 1;
static int goalY   = 7;

static int centerX = 2048;
static int centerY = 2048;
static int deadzone = 600;

static uint32_t lastBlink = 0;
static bool playerVisible = true;
static uint32_t lastMove = 0;

static bool xLatched = false;
static bool yLatched = false;

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t matrix_dev;

// ================== UTILS ==================
static inline uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}
static inline int clamp(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}
static inline bool inBounds(int x, int y) {
    return (x >= 0 && x < 8 && y >= 0 && y < 8);
}

// ================== ADC ONESHOT ==================
static adc_oneshot_unit_handle_t adc_unit;

static void adc_oneshot_init(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = JOY_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_unit));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,  // chip default
        .atten    = ADC_ATTEN_DB_11,       // 0-3.3V approx on S3
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_unit, JOY_VRX_CH, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_unit, JOY_VRY_CH, &chan_cfg));
}

static int adc_read_raw(adc_channel_t ch) {
    int val = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_unit, ch, &val));
    return val;
}

static int read_adc_avg(adc_channel_t ch, int samples) {
    int sum = 0;
    for (int i = 0; i < samples; i++) {
        int v = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_unit, ch, &v));
        sum += v;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return sum / samples;
}

// ================== BUTTON ==================
static void button_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << JOY_SW_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
}

// ================== GAME LOGIC ==================
static void initGoal(void) {
    for (int x = 0; x < 8; x++) {
        if(maze[7][x] == 0) {
            goalX = x;
            goalY = 7;
            return;
        }
    }
}

static void tryMove(int dx, int dy) {
    int nx = playerX + dx;
    int ny = playerY + dy;
    if (!inBounds(nx, ny)) return;
    if (maze[ny][nx] == 0) {
        playerX = nx;
        playerY = ny;
    }
}

static int endCondition(void) {
    int pressed = (gpio_get_level(JOY_SW_PIN) == 0); // active low
    return (playerX == goalX && playerY == goalY && pressed) ? 1 : 0;
}

static void drawScene(void) {
    // Build rows from mazeShow, then overlay player if visible
    uint8_t rows[8] = {0};

    for (int y = 0; y < 8; y++) {
        uint8_t row = 0;
        for ( int x = 0; x < 8; x++) {
            if (mazeShow[y][x]) {
                row |= (1u << x);
            }
        }
        rows[y] = row;
    }

    uint32_t now = millis();
    if (now - lastBlink >= BLINK_MS) {
        playerVisible = !playerVisible;
        lastBlink = now;
    }
    if (playerVisible) {
        rows[playerY] |= (1u << playerX);
    }
    ht16k33_write_rows(rows);
}

// ================== APP MAIN ==================
void app_main(void) {
    i2c_init_bus();
    if (!ht16k33_probe_and_init()) {
        printf("ERROR: No HT16K33 found at 0x70..0x77\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    adc_oneshot_init();
    button_init();

    // Deadzone ~18% of full scale, clamped
    deadzone = clamp((int)(0.18f * 4095.0f), DEADZONE_MIN, DEADZONE_MAX);

    initGoal();
    ht16k33_clear();
    drawScene();

    while (1) {
        uint32_t now = millis();

        int x = adc_read_raw(JOY_VRX_CH);
        int y = adc_read_raw(JOY_VRY_CH);
        int dx = x - centerX;
        int dy = y - centerY;

        if (endCondition() == 1) {
            ht16k33_clear();
            while (1) vTaskDelay(pdMS_TO_TICKS(1000)); // halt
        }

        if (abs(dx) > deadzone) {
            if (!xLatched && (now - lastMove) >= MOVE_COOLDOWN_MS) {
                if (dx < 0) tryMove(-1, 0); else tryMove(1,0);
                lastMove = now;
                xLatched = true;
            }
        } else {
            xLatched = false;
        }

        if (abs(dy) > deadzone) {
            if (!yLatched && (now - lastMove) >= MOVE_COOLDOWN_MS) {
                if (dy < 0) tryMove(0, -1); else tryMove(0,1);
                lastMove = now;
                yLatched = true;
            }
        } else {
            yLatched = false;
        }

        drawScene();
        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    }
}
