#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_check.h"

#define TAG "lcd_fix"

#define I2C_PORT      I2C_NUM_0
#define I2C_SDA_GPIO  12
#define I2C_SCL_GPIO  13
#define I2C_FREQ_HZ   100000

#define LCD_ADDR      0x3E  // AiP31068L text controller
#define RGB_ADDR      0x60  // PCA9633 RGB backlight controller

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t lcd, rgb;

// ---- Basic I2C helpers ----
static void i2c_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = { .enable_internal_pullup = true },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t lcd_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = LCD_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &lcd_cfg, &lcd));

    i2c_device_config_t rgb_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = RGB_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(bus, &rgb_cfg, &rgb) != ESP_OK) rgb = NULL;
}

// LCD low-level writes (AiP31068L control bytes)
static esp_err_t lcd_cmd(uint8_t cmd) {
    uint8_t b[2] = { 0x80, cmd };
    return i2c_master_transmit(lcd, b, 2, 50);
}
static esp_err_t lcd_data(const uint8_t *p, size_t n) {
    uint8_t tmp[1 + 32];
    while (n) {
        size_t c = n > 32 ? 32 : n;
        tmp[0] = 0x40;
        memcpy(&tmp[1], p, c);
        ESP_RETURN_ON_ERROR(i2c_master_transmit(lcd, tmp, c + 1, 50), TAG, "lcd tx");
        p += c; n -= c;
    }
    return ESP_OK;
}
static esp_err_t lcd_print(const char *s) { return lcd_data((const uint8_t*)s, strlen(s)); }

// AiP31068L “extended” init: enable booster/follower + contrast, then normal mode
static void lcd_init_aip31068(void) {
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_cmd(0x38);                        // function set (normal)
    lcd_cmd(0x39);                        // extended instruction set
    lcd_cmd(0x14);                        // internal osc freq (typical)
    uint8_t contrast = 0x35;              // 0x20..0x3F (tune if needed)
    lcd_cmd(0x70 | (contrast & 0x0F));    // Contrast[3:0]
    lcd_cmd(0x5C | ((contrast >> 4) & 0x03)); // Ion=1, Bon=1, Contrast[5:4]
    lcd_cmd(0x6C);                        // follower control ON
    vTaskDelay(pdMS_TO_TICKS(200));
    lcd_cmd(0x38);                        // back to normal instruction set
    lcd_cmd(0x0C);                        // display ON
    lcd_cmd(0x01);                        // clear
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_cmd(0x06);                        // entry mode: increment
}

// --- PCA9633: set MAX backlight (full white) ---
static void rgb_init_max_white(void) {
    if (!rgb) return;

    // MODE1: ensure normal mode (SLEEP=0)
    uint8_t mode1[] = { 0x00, 0x00 };
    i2c_master_transmit(rgb, mode1, sizeof(mode1), 50);

    // MODE2: totem-pole (OUTDRV=1), others default; 0x04 is typical
    uint8_t mode2[] = { 0x01, 0x04 };
    i2c_master_transmit(rgb, mode2, sizeof(mode2), 50);

    // LEDOUT: all channels PWM control
    uint8_t ledout[] = { 0x08, 0xAA };
    i2c_master_transmit(rgb, ledout, sizeof(ledout), 50);

    // PWM0..2 -> 255 (R,G,B full)
    uint8_t pwm0[] = { 0x02, 255 };
    uint8_t pwm1[] = { 0x03, 255 };
    uint8_t pwm2[] = { 0x04, 255 };
    i2c_master_transmit(rgb, pwm0, sizeof(pwm0), 50);
    i2c_master_transmit(rgb, pwm1, sizeof(pwm1), 50);
    i2c_master_transmit(rgb, pwm2, sizeof(pwm2), 50);
}

void app_main(void) {
    i2c_init();
    lcd_init_aip31068();
    rgb_init_max_white();                 // <— MAX backlight

    // Line 1, col 0
    lcd_cmd(0x80);
    lcd_print("Hi");

    // Line 2
    lcd_cmd(0x80 | 0x40);
    lcd_print("ESP32-S3 OK");

    ESP_LOGI(TAG, "Init done. Backlight at max.");
}
