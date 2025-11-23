// ESP-IDF v 5.5.1
#include "maze.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"


/* ========= CONFIG ========= */
// HT16K33 on shared I2C bus (bus handle passed from main)
#define MATRIX_BRIGHT        1       // 0..15
#define I2C_SPEED_HZ         50000   // 50 kHz

// Joystick: ADC1 channels (ESP32-S3) — GPIO1 -> CH0, GPIO2 -> CH1
#define JOY_ADC_UNIT         ADC_UNIT_1
#define JOY_VRX_CH           ADC_CHANNEL_0
#define JOY_VRY_CH           ADC_CHANNEL_1
#define JOY_SW_PIN           GPIO_NUM_3   // joystick/button (active low, pull-up)

// Movement tuning
#define MOVE_COOLDOWN_MS     200
#define DEADZONE_MIN         350
#define DEADZONE_MAX         1400
#define BLINK_MS             500

/* ========= INTERNAL STATE ========= */

static i2c_master_bus_handle_t  s_bus   = NULL;
static i2c_master_dev_handle_t  s_ht16k33 = NULL;
static adc_oneshot_unit_handle_t s_adc_unit = NULL;

static uint8_t s_ht16k33_addr = 0x70;

static const char *TAG = "MAZE";
static bool s_wallHitPending = false;

/* Maze data: 1 = wall, 0 = free */
static uint8_t s_maze[8][8] = {
    {1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,0,1},
    {1,1,1,1,0,1,0,1},
    {1,0,0,0,0,1,0,1},
    {1,1,1,0,1,1,0,1},
    {1,0,0,0,0,1,0,1},
    {1,0,1,1,0,0,0,1},
    {1,0,1,1,1,1,1,1}
};

/* Maze overlay */
static uint8_t s_mazeShow[8][8] = {
    {0,1,0,0,0,0,0,0},
    {1,0,0,0,0,0,0,0},
    {0,1,0,1,0,1,0,0},
    {0,0,0,0,0,1,0,0},
    {0,1,0,0,0,0,0,0},
    {0,0,0,0,0,1,0,0},
    {0,0,1,0,0,0,0,0},
    {0,0,0,0,0,0,0,0}
};


static int s_playerX = 1;
static int s_playerY = 1;
static int s_goalX   = 1;
static int s_goalY   = 7;

static int s_centerX = 2048;
static int s_centerY = 2048;
static int s_deadzone = 600;

static uint32_t s_lastBlink = 0;
static bool     s_playerVisible = true;
static uint32_t s_lastMove = 0;

static bool s_xLatched = false;
static bool s_yLatched = false;

static bool s_maze_running  = false;
static bool s_maze_finished = false;



/* ========= UTILS ========= */

static inline uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static inline int clamp_int(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline bool inBounds(int x, int y) {
    return (x >= 0 && x < 8 && y >= 0 && y < 8);
}

/* ========= LOW-LEVEL HELPERS ========= */
static bool ht16k33_probe_and_init(void) {
    i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address  = s_ht16k33_addr,
                .scl_speed_hz    = I2C_SPEED_HZ,
            };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_ht16k33));

    uint8_t cmd;
    cmd = 0x21;  ESP_ERROR_CHECK(i2c_master_transmit(s_ht16k33, &cmd, 1, -1)); // oscillator on
    cmd = 0x81;  ESP_ERROR_CHECK(i2c_master_transmit(s_ht16k33, &cmd, 1, -1)); // display on
    cmd = (uint8_t)(0xE0 | (MATRIX_BRIGHT & 0x0F));
    ESP_ERROR_CHECK(i2c_master_transmit(s_ht16k33, &cmd, 1, -1));              // brightness
    return true;
}


static void ht16k33_write_rows(uint8_t rows[8]) {
    if (!s_ht16k33) return;
    uint8_t buf[1 + 16];
    buf[0] = 0x00;
    for (int r = 0; r < 8; r++) {
        buf[1 + r*2]     = rows[r];
        buf[1 + r*2 + 1] = 0x00;
    }
    ESP_ERROR_CHECK(i2c_master_transmit(s_ht16k33, buf, sizeof(buf), -1));
}

static void ht16k33_clear(void) {
    uint8_t rows[8] = {0};
    ht16k33_write_rows(rows);
}


static void adc_oneshot_init_joystick(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = JOY_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_unit));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,  // chip default
        .atten    = ADC_ATTEN_DB_11,       // 0-3.3V approx on S3
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_unit, JOY_VRX_CH, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_unit, JOY_VRY_CH, &chan_cfg));
}

static int adc_read_raw_channel(adc_channel_t ch) {
    int val = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc_unit, ch, &val));
    return val;
}

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

/* ========= MAZE LOGIC ========= */

static void maze_initGoal(void) {
    for (int x = 0; x < 8; x++) {
        if (s_maze[7][x] == 0) {
            s_goalX = x;
            s_goalY = 7;
            return;
        }
    }
}

static void maze_tryMove(int dx, int dy) {
    int nx = s_playerX + dx;
    int ny = s_playerY + dy;

    // Out of bounds OR wall = wall-hit event
    if (!inBounds(nx, ny) || s_maze[ny][nx] != 0) {
        s_wallHitPending = true;
        return;
    }

    // Valid move
    s_playerX = nx;
    s_playerY = ny;
}

static int maze_endCondition(void) {
    int pressed = (gpio_get_level(JOY_SW_PIN) == 0); // active low
    return (s_playerX == s_goalX && s_playerY == s_goalY && pressed) ? 1 : 0;
}

static void maze_drawScene(void) {
    uint8_t rows[8] = {0};

    for (int y = 0; y < 8; y++) {
        uint8_t row = 0;
        for (int x = 0; x < 8; x++) {
            if (s_mazeShow[y][x]) {
                row |= (1u << x);
            }
        }
        rows[y] = row;
    }

    uint32_t now = millis();
    if (now - s_lastBlink >= BLINK_MS) {
        s_playerVisible = !s_playerVisible;
        s_lastBlink = now;
    }
    if (s_playerVisible) {
        rows[s_playerY] |= (1u << s_playerX);
    }
    ht16k33_write_rows(rows);
}

/* ========= PUBLIC API ========= */

void maze_init(i2c_master_bus_handle_t bus) {
    s_bus = bus;

    if (!ht16k33_probe_and_init()) {
        printf("Maze: ERROR: No HT16K33 found at 0x70..0x77\n");
        // Still continue; maze will just not draw
    }

    adc_oneshot_init_joystick();
    button_init();

    s_deadzone = clamp_int((int)(0.18f * 4095.0f), DEADZONE_MIN, DEADZONE_MAX);
    maze_initGoal();
    ht16k33_clear();
    maze_drawScene();

    s_maze_running  = false;
    s_maze_finished = false;

    ESP_LOGI(TAG, "HT16K33 device added");
}

void maze_start(void) {
    s_playerX = 1;
    s_playerY = 1;
    maze_initGoal();

    s_centerX = 2048;
    s_centerY = 2048;
    s_lastBlink = millis();
    s_lastMove  = millis();
    s_xLatched  = false;
    s_yLatched  = false;
    s_playerVisible = true;

    s_maze_finished = false;
    s_maze_running  = true;

    s_wallHitPending = false;

    ht16k33_clear();
    maze_drawScene();
}

void maze_update(void) {
    if (!s_maze_running || s_maze_finished) return;
    uint32_t now = millis();

    int x = adc_read_raw_channel(JOY_VRX_CH);
    int y = adc_read_raw_channel(JOY_VRY_CH);
    int dx = x - s_centerX;
    int dy = y - s_centerY;

    if (maze_endCondition() == 1) {
        s_maze_finished = true;
        s_maze_running  = false;
        ht16k33_clear();
        return;
    }

    if (abs(dx) > s_deadzone) {
        if (!s_xLatched && (now - s_lastMove) >= MOVE_COOLDOWN_MS) {
            if (dx < 0) maze_tryMove(-1, 0); else maze_tryMove(1, 0);
            s_lastMove = now;
            s_xLatched = true;
        }
    } else {
        s_xLatched = false;
    }

    if (abs(dy) > s_deadzone) {
        if (!s_yLatched && (now - s_lastMove) >= MOVE_COOLDOWN_MS) {
            if (dy < 0) maze_tryMove(0, -1); else maze_tryMove(0, 1);
            s_lastMove = now;
            s_yLatched = true;
        }
    } else {
        s_yLatched = false;
    }

    maze_drawScene();
}

void maze_stop(void) {
    s_maze_running = false;
    ht16k33_clear();
    s_wallHitPending = false;
}

bool maze_is_finished(void) {
    return s_maze_finished;
}

bool maze_poll_wall_hit(void)
{
    if (s_wallHitPending) {
        s_wallHitPending = false;  // one-shot
        return true;
    }
    return false;
}