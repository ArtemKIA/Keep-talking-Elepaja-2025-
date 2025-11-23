// freq.c  -- 2.4" SPI TFT (ILI9341-style) + 3 potentiometers on GPIO15/16/17 (ADC2)

#include "freq.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "FREQ";

/* ==== DISPLAY CONFIG (2.4" TFT, ILI9341-style, 240x320, 4-wire SPI) ==== */

#define TFT_W 240
#define TFT_H 320

// SPI host
#define TFT_SPI_HOST     SPI2_HOST

// SPI pins (change if rewired)
#define TFT_PIN_SCLK     GPIO_NUM_37
#define TFT_PIN_MOSI     GPIO_NUM_38
#define TFT_PIN_CS       GPIO_NUM_11
#define TFT_PIN_DC       GPIO_NUM_12
#define TFT_PIN_RST      GPIO_NUM_8

// RGB565 colors
#define COLOR_BLACK      0x0000
#define COLOR_GRAY       0x8410
#define COLOR_YELLOW     0xFFE0

/* ==== WAVEFORM PARAMS ==== */

static const float K_TARGET = 3.0f;
static const float A_REF    = 80.0f;
static const int   CY       = TFT_H / 2;

#define K_TOL              0.05f
#define MATCH_HOLD_MS      1000

/* ==== POTENTIOMETERS ==== */

#define POT_ADC_UNIT       ADC_UNIT_2
#define POT1_CH            ADC_CHANNEL_4   // GPIO15
#define POT2_CH            ADC_CHANNEL_5   // GPIO16
#define POT3_CH            ADC_CHANNEL_6   // GPIO17

/* ==== INTERNAL ==== */

static spi_device_handle_t        s_tft       = NULL;
static adc_oneshot_unit_handle_t  s_adc2      = NULL;

static float    s_k            = K_TARGET;
static bool     s_running      = false;
static bool     s_matched      = false;
static uint32_t s_match_start  = 0;

/* ==== TIME ==== */

static inline uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ==== ADC ==== */

static void freq_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = POT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc2));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_11,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc2, POT1_CH, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc2, POT2_CH, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc2, POT3_CH, &ch_cfg));

    ESP_LOGI(TAG, "ADC2 init for GPIO15/16/17");
}

static void freq_adc_read_all(int *p1, int *p2, int *p3)
{
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc2, POT1_CH, p1));
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc2, POT2_CH, p2));
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc2, POT3_CH, p3));
}

/* ==== SPI LOW-LEVEL ==== */

static esp_err_t tft_write_cmd(uint8_t cmd)
{
    spi_transaction_t t = {0};
    t.length    = 8;
    t.tx_buffer = &cmd;
    gpio_set_level(TFT_PIN_DC, 0);
    return spi_device_transmit(s_tft, &t);
}

static esp_err_t tft_write_data(const uint8_t *data, size_t len)
{
    spi_transaction_t t = {0};
    t.length    = len * 8;
    t.tx_buffer = data;
    gpio_set_level(TFT_PIN_DC, 1);
    return spi_device_transmit(s_tft, &t);
}

static inline esp_err_t tft_write_data_byte(uint8_t v)
{
    return tft_write_data(&v, 1);
}

static void tft_hw_reset(void)
{
#if TFT_PIN_RST >= 0
    gpio_set_direction(TFT_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(TFT_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TFT_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
#endif
}

static void tft_init_spi(void)
{
    gpio_set_direction(TFT_PIN_DC, GPIO_MODE_OUTPUT);

    spi_bus_config_t buscfg = {
        .mosi_io_num     = TFT_PIN_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = TFT_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = TFT_W * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TFT_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = TFT_PIN_CS,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(TFT_SPI_HOST, &devcfg, &s_tft));

    ESP_LOGI(TAG, "SPI TFT ready");
}

static void tft_init_device(void)
{
    tft_hw_reset();

    tft_write_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(120));

    tft_write_cmd(0x28);

    tft_write_cmd(0x3A);
    tft_write_data_byte(0x55);

    tft_write_cmd(0x36);
    tft_write_data_byte(0x48);

    tft_write_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    tft_write_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "TFT init done");
}

static void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    tft_write_cmd(0x2A);
    data[0] = x0 >> 8; data[1] = x0;
    data[2] = x1 >> 8; data[3] = x1;
    tft_write_data(data, 4);

    tft_write_cmd(0x2B);
    data[0] = y0 >> 8; data[1] = y0;
    data[2] = y1 >> 8; data[3] = y1;
    tft_write_data(data, 4);

    tft_write_cmd(0x2C);
}

static void tft_draw_pixel(int x, int y, uint16_t color)
{
    if ((unsigned)x >= TFT_W || (unsigned)y >= TFT_H) return;

    tft_set_addr_window(x, y, x, y);

    uint8_t data[2];
    data[0] = color >> 8;
    data[1] = color;
    tft_write_data(data, 2);
}

static void tft_fill_screen(uint16_t color)
{
    tft_set_addr_window(0, 0, TFT_W - 1, TFT_H - 1);

    uint8_t line[TFT_W * 2];
    for (int i = 0; i < TFT_W; ++i) {
        line[2 * i]     = color >> 8;
        line[2 * i + 1] = color;
    }

    for (int y = 0; y < TFT_H; ++y) {
        tft_write_data(line, sizeof(line));
    }
}

static void tft_draw_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1) {
        tft_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* ==== WAVE DRAWING ==== */

static void draw_wave_frame(float k, float amp, float phi)
{
    static uint32_t s_last_draw = 0;
    uint32_t now = millis();

    // Lower frame rate: ~5 FPS instead of 10+
    const uint32_t FRAME_INTERVAL_MS = 200;
    if ((now - s_last_draw) < FRAME_INTERVAL_MS) {
        return;
    }
    s_last_draw = now;

    // 1) CLEAR BACKGROUND ONCE PER FRAME
    tft_fill_screen(COLOR_BLACK);

    int prevx = 0;
    float theta0 = 2.0f * M_PI * (k * 0.0f / TFT_W) + phi;
    int prevy = CY - (int)lrintf(amp * sinf(theta0));

    // 2) DRAW FEWER POINTS (step 2 pixels horizontally)
    for (int x = 0; x < TFT_W; x += 2) {

        // dotted reference wave
        float theta_ref = 2.0f * M_PI * (K_TARGET * x / TFT_W);
        int y_ref = CY - (int)lrintf(A_REF * sinf(theta_ref));
        if ((x % 6) == 0 && (unsigned)y_ref < TFT_H) {   // coarser dots
            tft_draw_pixel(x, y_ref, COLOR_GRAY);
        }

        // live wave
        float theta = 2.0f * M_PI * (k * x / TFT_W) + phi;
        int y = CY - (int)lrintf(amp * sinf(theta));

        if ((unsigned)y < TFT_H || (unsigned)prevy < TFT_H) {
            tft_draw_line(prevx, prevy, x, y, COLOR_YELLOW);
        }

        prevx = x;
        prevy = y;
    }
}

/* ==== PUBLIC API ==== */

void freq_init(void)
{
    freq_adc_init();
    tft_init_spi();
    tft_init_device();

    s_k           = K_TARGET;
    s_running     = false;
    s_matched     = false;
    s_match_start = 0;

    tft_fill_screen(COLOR_BLACK);

    ESP_LOGI(TAG, "Frequency module init");
}

void freq_start(void)
{
    s_running     = true;
    s_matched     = false;
    s_match_start = 0;
    ESP_LOGI(TAG, "Frequency module start");
}

void freq_stop(void)
{
    s_running = false;
    ESP_LOGI(TAG, "Frequency module stop");
}

bool freq_is_matched(void)
{
    return s_matched;
}

void freq_update(void)
{
    if (!s_running) return;

    int r1, r2, r3;
    freq_adc_read_all(&r1, &r2, &r3);

    float k_target = 0.25f + 5.75f * (float)r1 / 4095.0f;
    float amp      = 20.0f + 60.0f * (float)r2 / 4095.0f;
    float phi      = 2.0f * M_PI * (float)r3 / 4095.0f;

    const float alpha = 0.15f;
    s_k = (1.0f - alpha) * s_k + alpha * k_target;

    float diff    = s_k - K_TARGET;
    float err_abs = fabsf(diff);
    uint32_t now  = millis();

    if (err_abs <= K_TOL) {
        if (s_match_start == 0) {
            s_match_start = now;
        } else if ((now - s_match_start) >= MATCH_HOLD_MS) {
            s_matched = true;
        }
    } else {
        s_match_start = 0;
        s_matched     = false;
    }

    draw_wave_frame(s_k, amp, phi);
}

/* ==== BACKGROUND TASK ==== */

static void freq_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_running) {
            freq_update();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void freq_start_task(void)
{
    xTaskCreatePinnedToCore(
        freq_task,
        "freq_task",
        4096,
        NULL,
        tskIDLE_PRIORITY,   // 0: strictly background
        NULL,
        1                   // keep on core 1 if you want
    );
}