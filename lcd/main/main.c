#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/gpio.h"

#define Y_CHANNEL ADC1_CHANNEL_6 // GPIO34
#define BUTTON GPIO_NUM_12

#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_11
#define DEADZONE 100
#define SCROLL   1500

typedef enum {
    SCROLL_UP,
    SCROLL_DOWN,
    NO_SCROLL,
    PRESS
} joystick_event_t;

static const char *TAG = "MAIN";

/* I2C / LCD */
#define I2C_BUS_PORT 0
#define PIN_NUM_SDA 21
#define PIN_NUM_SCL 22
#define LCD_ADDRESS 0x3E
#define RGB_ADDRESS 0x60
#define I2C_MASTER_FREQ_HZ (50 * 1000) // 100kHz

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t lcd_dev_handle = NULL;
static i2c_master_dev_handle_t rgb_dev_handle = NULL;

/* Menu */
#define MENU_ITEMS 4
const char *menu[MENU_ITEMS] = {
    "Option 1",
    "Option 2",
    "Option 3",
    "Option 4"
};
static int current_index = 0;
static int top_index = 0;

/* Forward declarations */
static esp_err_t i2c_init_and_devices(void);
static esp_err_t lcd_init_display(void);

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
    static int16_t last_y_value = 2048;
    int16_t y_value = adc1_get_raw(Y_CHANNEL);
    int button_state = gpio_get_level(BUTTON);

    if (button_state == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(BUTTON) == 0) return PRESS;
    }

    int16_t y_diff = y_value - 2048;
    if (abs(y_diff) < DEADZONE) {
        last_y_value = y_value;
        return NO_SCROLL;
    }

    if (y_diff < -SCROLL && last_y_value > (2048 - SCROLL/2)) {
        last_y_value = y_value; return SCROLL_UP;
    } else if (y_diff > SCROLL && last_y_value < (2048 + SCROLL/2)) {
        last_y_value = y_value; return SCROLL_DOWN;
    }
    last_y_value = y_value;
    return NO_SCROLL;
}

/* ------------------ I2C + Device setup + recovery ------------------ */
static esp_err_t i2c_init_bus(void) {
    if (i2c_bus) {
        // already created
        return ESP_OK;
    }
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = PIN_NUM_SDA,
        .scl_io_num = PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: 0x%X", ret);
    }
    return ret;
}

static esp_err_t lcd_add_device(void) {
    if (!i2c_bus) return ESP_ERR_INVALID_STATE;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = LCD_ADDRESS,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, &lcd_dev_handle);
}

static esp_err_t rgb_add_device_and_init(void) {
    if (!i2c_bus) return ESP_ERR_INVALID_STATE;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = RGB_ADDRESS,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t r = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &rgb_dev_handle);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add RGB device: 0x%X", r);
        return r;
    }

    uint8_t init_data[][2] = {
        {0x80, 0x01},
        {0x81, 0x14},
        {0x82, 0xFF},
        {0x83, 0xFF},
        {0x84, 0xFF},
        {0x85, 0x20}
    };

    for (size_t i = 0; i < sizeof(init_data)/sizeof(init_data[0]); ++i) {
        esp_err_t t = i2c_master_transmit(rgb_dev_handle, init_data[i], 2, pdMS_TO_TICKS(100));
        if (t != ESP_OK) {
            ESP_LOGW(TAG, "RGB init reg 0x%02X failed: 0x%X", init_data[i][0], t);
            // continue trying other regs
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return ESP_OK;
}

/* Attempt to re-create bus and devices on error. This helps clear a stuck bus. */
static void i2c_recover_and_reinit(void) {
    ESP_LOGW(TAG, "Attempting I2C recovery/reinit...");
    // delete existing bus handle if any
    if (i2c_bus) {
        // no direct API to delete bus in some IDF versions, so set handles to NULL and try to re-create
        // In practice you might want to call i2c_del_master or restart device; here we'll try simple reinit steps:
        i2c_bus = NULL;
        lcd_dev_handle = NULL;
        rgb_dev_handle = NULL;
    }
    // short delay and reinit
    vTaskDelay(pdMS_TO_TICKS(100));
    if (i2c_init_and_devices() != ESP_OK) {
        ESP_LOGE(TAG, "I2C recovery failed");
    } else {
        ESP_LOGI(TAG, "I2C recovered");
    }
}

/* init bus + devices + lcd display */
static esp_err_t i2c_init_and_devices(void) {
    esp_err_t r = i2c_init_bus();
    if (r != ESP_OK) return r;

    r = lcd_add_device();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "lcd_add_device failed: 0x%X", r);
        return r;
    }
    r = rgb_add_device_and_init();
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "rgb init returned 0x%X", r);
    }
    // initialize display
    r = lcd_init_display();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "lcd_init_display failed: 0x%X", r);
    }
    return r;
}

/* ------------------ LCD helpers ------------------ */
/* Return esp_err_t so caller can handle failures */
static esp_err_t lcd_cmd(uint8_t cmd) {
    if (!lcd_dev_handle) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = {0x00, cmd};
    esp_err_t r = i2c_master_transmit(lcd_dev_handle, buf, 2, pdMS_TO_TICKS(100));
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "lcd_cmd 0x%02X failed: 0x%X", cmd, r);
    }
    return r;
}

static esp_err_t lcd_data(uint8_t data) {
    if (!lcd_dev_handle) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = {0x40, data};
    esp_err_t r = i2c_master_transmit(lcd_dev_handle, buf, 2, pdMS_TO_TICKS(100));
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "lcd_data 0x%02X failed: 0x%X", data, r);
    }
    return r;
}

/* AiP31068 specific init sequence */
static esp_err_t lcd_init_display(void) {
    // If lcd_dev_handle is not ready, return error
    if (!lcd_dev_handle) return ESP_ERR_INVALID_STATE;

    vTaskDelay(pdMS_TO_TICKS(50));
    if (lcd_cmd(0x38) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(5));
    if (lcd_cmd(0x39) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(5));
    if (lcd_cmd(0x14) != ESP_OK) return ESP_FAIL;
    if (lcd_cmd(0x70 | 0x0F) != ESP_OK) return ESP_FAIL;
    if (lcd_cmd(0x5C) != ESP_OK) return ESP_FAIL;
    if (lcd_cmd(0x6C) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(200));
    if (lcd_cmd(0x38) != ESP_OK) return ESP_FAIL;
    if (lcd_cmd(0x0C) != ESP_OK) return ESP_FAIL;
    if (lcd_cmd(0x01) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_LOGI(TAG, "LCD initialised");
    return ESP_OK;
}

/* Write string with small spacing and recovery on error */
static void lcd_write_str(const char *str) {
    int fail_count = 0;
    while (*str) {
        esp_err_t r = lcd_data((uint8_t)*str++);
        vTaskDelay(pdMS_TO_TICKS(2));
        if (r != ESP_OK) {
            fail_count++;
            if (fail_count > 2) {
                // try recovery procedure
                ESP_LOGE(TAG, "Multiple LCD write failures, trying I2C recovery");
                i2c_recover_and_reinit();
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            fail_count = 0;
            vTaskDelay(pdMS_TO_TICKS(2)); // tiny delay so device isn't flooded
        }
    }
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0 ? 0x00 : 0x40) + col;
    lcd_cmd(0x80 | addr);
}

static void lcd_render_menu(void) {
    if (lcd_cmd(0x01) != ESP_OK) { ESP_LOGW(TAG,"clear failed"); }
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_set_cursor(0, 0);
    if (top_index < MENU_ITEMS) {
        lcd_write_str((top_index == current_index) ? ">" : " ");
        lcd_write_str(menu[top_index]);
    }

    lcd_set_cursor(1, 0);
    if (top_index + 1 < MENU_ITEMS) {
        lcd_write_str((top_index + 1 == current_index) ? ">" : " ");
        lcd_write_str(menu[top_index + 1]);
    }
}

static void set_backlight_color(uint8_t red, uint8_t green, uint8_t blue) {
    if (!rgb_dev_handle) return;
    uint8_t red_cmd[]   = {0x82, red};
    uint8_t green_cmd[] = {0x83, green};
    uint8_t blue_cmd[]  = {0x84, blue};
    i2c_master_transmit(rgb_dev_handle, red_cmd, 2, pdMS_TO_TICKS(100));
    i2c_master_transmit(rgb_dev_handle, green_cmd, 2, pdMS_TO_TICKS(100));
    i2c_master_transmit(rgb_dev_handle, blue_cmd, 2, pdMS_TO_TICKS(100));
}

/* ------------------ joystick/menu task ------------------ */
void joystick_task(void *pvParameters) {
    joystick_init();
    lcd_render_menu();

    while (1) {
        joystick_event_t event = joystick_read_event();

        switch (event) {
            case SCROLL_UP:
                if (current_index > 0) {
                    current_index--;
                    if (current_index < top_index) top_index--;
                    lcd_render_menu();
                }
                break;
            case SCROLL_DOWN:
                if (current_index < MENU_ITEMS - 1) {
                    current_index++;
                    if (current_index > top_index + 1) top_index++;
                    lcd_render_menu();
                }
                break;
            case PRESS:
                ESP_LOGI(TAG, "Selected: %s", menu[current_index]);
                lcd_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(2));
                lcd_set_cursor(0, 0); lcd_write_str("Selected:");
                lcd_set_cursor(1, 0); lcd_write_str(menu[current_index]);
                vTaskDelay(pdMS_TO_TICKS(1500));
                lcd_render_menu();
                break;
            case NO_SCROLL:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ------------------ main ------------------ */
void app_main(void) {
    ESP_LOGI(TAG, "Starting");

    if (i2c_init_and_devices() != ESP_OK) {
        ESP_LOGE(TAG, "Initial i2c/dev init failed, continuing (you may want to reset)");
    }

    set_backlight_color(0xFF, 0xFF, 0xFF);

    // initial text (safe small burst)
    lcd_set_cursor(0,0);
    lcd_write_str("Hello Bitches!");
    lcd_set_cursor(1,0);
    lcd_write_str("I AM BACK");

    xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 5, NULL);
}
