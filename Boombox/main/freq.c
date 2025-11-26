// freq.c  -- 0.96" I2C OLED (SSD1306 128x64) + 3 potentiometers on GPIO15/16/17 (ADC2)
// P1 = amplitude, P2 = period/frequency, P3 = noise amount

#include "freq.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_random.h"

/* ==== DISPLAY CONFIG (SSD1306 128x64, I2C) ==== */

#define OLED_W              128
#define OLED_H              64
#define OLED_ADDR           0x3C
#define OLED_I2C_SPEED_HZ   (400 * 1000)

/* framebuffer: 8 pages x 128 columns, 1 bit per pixel (page/column) */
static uint8_t s_fb[OLED_H / 8][OLED_W];

/* ==== WAVEFORM PARAMS ==== */

static const float K_TARGET = 3.0f;
/* reference wave amplitude tuned for 64 px height */
static const float A_REF    = 24.0f;
static const int   CY       = OLED_H / 2;

#define K_TOL              0.05f
#define MATCH_HOLD_MS      1000

/* ==== POTENTIOMETERS ==== */

#define POT_ADC_UNIT       ADC_UNIT_2
#define POT1_CH            ADC_CHANNEL_4   // GPIO15 - amplitude
#define POT2_CH            ADC_CHANNEL_5   // GPIO16 - period/frequency
#define POT3_CH            ADC_CHANNEL_6   // GPIO17 - noise amount

/* ==== INTERNAL STATE ==== */

static const char *TAG = "FREQ";

static i2c_master_bus_handle_t   s_bus   = NULL;
static i2c_master_dev_handle_t   s_oled  = NULL;
static adc_oneshot_unit_handle_t s_adc2  = NULL;

static float    s_k           = K_TARGET;
static bool     s_running     = false;
static bool     s_matched     = false;
static uint32_t s_match_start = 0;

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

/* ==== SSD1306 LOW-LEVEL ==== */

static inline esp_err_t oled_send_cmd(uint8_t cmd)
{
    if (!s_oled) return ESP_FAIL;
    uint8_t buf[2] = { 0x00, cmd };  // control byte 0x00 = command
    return i2c_master_transmit(s_oled, buf, sizeof(buf), -1);
}

static inline esp_err_t oled_send_data(const uint8_t *data, size_t len)
{
    if (!s_oled) return ESP_FAIL;

    if (len > OLED_W) len = OLED_W;

    uint8_t buf[1 + OLED_W];
    buf[0] = 0x40; // control byte 0x40 = data
    memcpy(&buf[1], data, len);

    return i2c_master_transmit(s_oled, buf, 1 + len, -1);
}

static void oled_flush(void)
{
    if (!s_oled) return;

    for (int page = 0; page < (OLED_H / 8); ++page) {
        uint8_t cmd_buf[4] = {
            0x00,
            (uint8_t)(0xB0 | page),   // page address
            0x00,                     // low column
            0x10                      // high column
        };
        ESP_ERROR_CHECK(i2c_master_transmit(s_oled, cmd_buf, sizeof(cmd_buf), -1));
        ESP_ERROR_CHECK(oled_send_data(s_fb[page], OLED_W));
    }
}

static void oled_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

/* ==== FRAMEBUFFER DRAW ==== */

static void oled_draw_pixel(int x, int y, bool on)
{
    if ((unsigned)x >= OLED_W || (unsigned)y >= OLED_H) return;

    int page = y >> 3;
    uint8_t bit = (uint8_t)(1u << (y & 7));

    if (on) {
        s_fb[page][x] |= bit;
    } else {
        s_fb[page][x] &= (uint8_t)~bit;
    }
}

static void oled_draw_line(int x0, int y0, int x1, int y1, bool on)
{
    int dx = abs(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1) {
        oled_draw_pixel(x0, y0, on);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* ==== SSD1306 INIT ==== */

static void oled_init_i2c(i2c_master_bus_handle_t bus)
{
    s_bus = bus;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_ADDR,
        .scl_speed_hz    = OLED_I2C_SPEED_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_oled));

    ESP_LOGI(TAG, "SSD1306 OLED added at 0x%02X", OLED_ADDR);
}

static void oled_init_device(void)
{
    static const uint8_t init_seq[] = {
        0xAE,          // display off
        0xD5, 0x80,    // clock divide
        0xA8, 0x3F,    // multiplex: 1/64
        0xD3, 0x00,    // display offset
        0x40,          // start line = 0
        0x8D, 0x14,    // charge pump enable
        0x20, 0x00,    // memory mode: horizontal
        0xA1,          // segment remap
        0xC8,          // COM scan direction
        0xDA, 0x12,    // COM pins config
        0x81, 0xCF,    // contrast
        0xD9, 0xF1,    // pre-charge
        0xDB, 0x40,    // VCOM detect
        0xA4,          // display follows RAM
        0xA6,          // normal display
        0xAF           // display on
    };

    for (size_t i = 0; i < sizeof(init_seq); ++i) {
        ESP_ERROR_CHECK(oled_send_cmd(init_seq[i]));
    }

    oled_clear();
    oled_flush();

    ESP_LOGI(TAG, "SSD1306 init done (128x64)");
}

/* ==== WAVE DRAWING ==== */
/* k      = cycles across screen (controls period/frequency)
 * amp    = sine amplitude in pixels
 * noiseA = 0..1, scales vertical noise added to player wave
 */
static void draw_wave_frame(float k, float amp, float noiseA)
{
    static uint32_t s_last_draw = 0;
    uint32_t now = millis();

    /* ~5 FPS */
    const uint32_t FRAME_INTERVAL_MS = 200;
    if ((now - s_last_draw) < FRAME_INTERVAL_MS) {
        return;
    }
    s_last_draw = now;

    oled_clear();

    /* slow automatic phase to make the wave move */
    float t   = (float)now / 1000.0f;
    float phi = 2.0f * (float)M_PI * 0.25f * t; // 0.25 Hz scroll

    int prevx = 0;
    float theta0 = 2.0f * (float)M_PI * (k * 0.0f / (float)OLED_W) + phi;
    float noise_px_max = noiseA * 10.0f;        // up to ~±10 px at max noise
    float y0 = amp * sinf(theta0);
    if (noise_px_max > 0.0f) {
        int32_t r = (int32_t)(esp_random() & 0xFFFF);
        float u = (float)r / 32768.0f - 1.0f;   // approx -1..+1
        y0 += noise_px_max * u;
    }
    int prevy = CY - (int)lrintf(y0);

    for (int x = 0; x < OLED_W; ++x) {

        /* dotted reference wave (clean, no noise) */
        float theta_ref = 2.0f * (float)M_PI * (K_TARGET * (float)x / (float)OLED_W);
        int y_ref = CY - (int)lrintf(A_REF * sinf(theta_ref));
        if ((x % 6) == 0 && (unsigned)y_ref < OLED_H) {
            oled_draw_pixel(x, y_ref, true);
        }

        /* live wave with noise */
        float theta = 2.0f * (float)M_PI * (k * (float)x / (float)OLED_W) + phi;
        float y_f   = amp * sinf(theta);

        if (noise_px_max > 0.0f) {
            int32_t r = (int32_t)(esp_random() & 0xFFFF);
            float u = (float)r / 32768.0f - 1.0f;   // approx -1..+1
            y_f += noise_px_max * u;
        }

        int y = CY - (int)lrintf(y_f);

        if ((unsigned)y < OLED_H || (unsigned)prevy < OLED_H) {
            oled_draw_line(prevx, prevy, x, y, true);
        }

        prevx = x;
        prevy = y;
    }

    oled_flush();
}

/* ==== PUBLIC API ==== */

void freq_init(i2c_master_bus_handle_t bus)
{
    freq_adc_init();
    oled_init_i2c(bus);
    oled_init_device();

    s_k           = K_TARGET;
    s_running     = false;
    s_matched     = false;
    s_match_start = 0;

    ESP_LOGI(TAG, "Frequency module init (I2C OLED 128x64)");
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

    /* P1: amplitude mapped to display height (avoid clipping) */
    float maxAmp = (float)(CY - 4);         // leave margin
    float amp    = 4.0f + (maxAmp - 4.0f) * (float)r1 / 4095.0f;

    /* P2: k from ~0.25..6.0 (controls period/frequency) */
    float k_target = 0.25f + 5.75f * (float)r2 / 4095.0f;

    /* P3: noise amount 0..1 */
    float noiseA = (float)r3 / 4095.0f;

    /* low-pass filter k for smoother motion */
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

    draw_wave_frame(s_k, amp, noiseA);
}
