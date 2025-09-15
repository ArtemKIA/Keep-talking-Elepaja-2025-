#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/gpio.h"

/* ------------------ CONFIG ------------------ */
#define Y_CHANNEL ADC1_CHANNEL_4  // GPIO34 on ESP32 || GPIO8 on ESP32-S3
#define BUTTON    GPIO_NUM_9      // ESP32-S3 pin 46

#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_11
#define DEADZONE  300
#define SCROLL    2000

#define I2C_BUS_PORT 0
#define PIN_NUM_SDA 13
#define PIN_NUM_SCL 14
#define LCD_ADDRESS 0x3E
#define RGB_ADDRESS 0x60
#define I2C_MASTER_FREQ_HZ (50 * 1000)

static const char *TAG = "MAIN";

/* ------------------ MENU ------------------ */
typedef enum {
    MENU_MAIN,
    MENU_PLAY,
    MENU_DIFFICULTY,
    MENU_TIME,
    MENU_OPTION5
} menu_state_t;

typedef enum {
    SCROLL_UP,
    SCROLL_DOWN,
    NO_SCROLL,
    PRESS
} joystick_event_t;

#define MENU_ITEMS 5
const char *menu[MENU_ITEMS] = {
    "Play",
    "Difficulty",
    "Time",
    "Option 5",
    "Exit"
};

static menu_state_t menu_state = MENU_MAIN;
static int current_index = 0;
static int top_index = 0;

static int game_time = 60; // default = 1 min
static int difficulty = 0;
const char *difficulty_labels[3] = {"Easy", "Medium", "Hard"};

/* ------------------ I2C / LCD ------------------ */
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t lcd_dev_handle = NULL;
static i2c_master_dev_handle_t rgb_dev_handle = NULL;

/* ------------------ Helpers ------------------ */
static void format_time(int seconds, char *buf, size_t len) {
    int minutes = seconds / 60;
    int secs = seconds % 60;
    snprintf(buf, len, "%d:%02d", minutes, secs);
}

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

/* ------------------ I2C + Devices ------------------ */
static esp_err_t i2c_init_bus(void) {
    if (i2c_bus) return ESP_OK;

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = PIN_NUM_SDA,
        .scl_io_num = PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &i2c_bus);
}

static esp_err_t lcd_add_device(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = LCD_ADDRESS,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, &lcd_dev_handle);
}

static esp_err_t rgb_add_device_and_init(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = RGB_ADDRESS,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t r = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &rgb_dev_handle);
    if (r != ESP_OK) return r;

    uint8_t init_data[][2] = {
        {0x80, 0x01},{0x81, 0x14},{0x82, 0xFF},
        {0x83, 0xFF},{0x84, 0xFF},{0x85, 0x20}
    };

    for (size_t i = 0; i < sizeof(init_data)/sizeof(init_data[0]); i++) {
        i2c_master_transmit(rgb_dev_handle, init_data[i], 2, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK;
}

/* ------------------ LCD helpers ------------------ */
static esp_err_t lcd_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    esp_err_t r = i2c_master_transmit(lcd_dev_handle, buf, 2, pdMS_TO_TICKS(100));
    if (r == ESP_OK) {
        if (cmd == 0x01 || cmd == 0x02) vTaskDelay(pdMS_TO_TICKS(5));
        else vTaskDelay(pdMS_TO_TICKS(2));
    }
    return r;
}

static esp_err_t lcd_data(uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    return i2c_master_transmit(lcd_dev_handle, buf, 2, pdMS_TO_TICKS(100));
}

static void lcd_write_str(const char *str) {
    while (*str) {
        lcd_data((uint8_t)*str++);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* New: write full padded line */
static void lcd_write_line(uint8_t row, const char *text) {
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16s", text); // left align, pad spaces
    uint8_t addr = (row == 0 ? 0x00 : 0x40);
    lcd_cmd(0x80 | addr);
    lcd_write_str(buf);
}

static esp_err_t lcd_init_display(void) {
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_cmd(0x38); lcd_cmd(0x39); lcd_cmd(0x14);
    lcd_cmd(0x70 | 0x0F); lcd_cmd(0x5C); lcd_cmd(0x6C);
    vTaskDelay(pdMS_TO_TICKS(200));
    lcd_cmd(0x38); lcd_cmd(0x0C); lcd_cmd(0x01);
    ESP_LOGI(TAG, "LCD initialised");
    return ESP_OK;
}

static void lcd_render_menu(void) {
    char line0[17], line1[17];
    snprintf(line0, sizeof(line0), "%c%s", (top_index==current_index)?'>':' ', menu[top_index]);
    lcd_write_line(0, line0);

    if (top_index+1 < MENU_ITEMS) {
        snprintf(line1, sizeof(line1), "%c%s", (top_index+1==current_index)?'>':' ', menu[top_index+1]);
        lcd_write_line(1, line1);
    } else {
        lcd_write_line(1, "");
    }
}

/* ------------------ Menu Task ------------------ */
void joystick_task(void *pvParameters) {
    joystick_init();
    lcd_render_menu();

    while (1) {
        joystick_event_t event = joystick_read_event();

        switch (menu_state) {
        case MENU_MAIN:
            if (event == SCROLL_UP && current_index > 0) {
                current_index--;
                if (current_index < top_index) top_index--;
                lcd_render_menu();
            } else if (event == SCROLL_DOWN && current_index < MENU_ITEMS-1) {
                current_index++;
                if (current_index > top_index+1) top_index++;
                lcd_render_menu();
            } else if (event == PRESS) {
                if (current_index == 0) menu_state = MENU_PLAY;
                else if (current_index == 1) menu_state = MENU_DIFFICULTY;
                else if (current_index == 2) menu_state = MENU_TIME;
                else if (current_index == 3) menu_state = MENU_OPTION5;
                else if (current_index == 4) {
                    lcd_write_line(0, "Goodbye!");
                    lcd_write_line(1, "");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
            }
            break;

        case MENU_PLAY:
            lcd_write_line(0, "Game Starting...");
            lcd_write_line(1, "");
            vTaskDelay(pdMS_TO_TICKS(2000));
            menu_state = MENU_MAIN;
            lcd_render_menu();
            break;

        case MENU_DIFFICULTY:
            if (event == SCROLL_UP && difficulty < 2) difficulty++;
            else if (event == SCROLL_DOWN && difficulty > 0) difficulty--;
            else if (event == PRESS) { menu_state = MENU_MAIN; lcd_render_menu(); break; }

            lcd_write_line(0, "Difficulty:");
            lcd_write_line(1, difficulty_labels[difficulty]);
            break;

        case MENU_TIME: {
            if (event == SCROLL_UP && game_time < 300) game_time += 30;
            else if (event == SCROLL_DOWN && game_time > 60) game_time -= 30;
            else if (event == PRESS) { menu_state = MENU_MAIN; lcd_render_menu(); break; }

            char buf[16];
            format_time(game_time, buf, sizeof(buf));
            lcd_write_line(0, "Set Time:");
            lcd_write_line(1, buf);
            break;
        }

        case MENU_OPTION5:
            lcd_write_line(0, "Option 5 TBD");
            lcd_write_line(1, "");
            vTaskDelay(pdMS_TO_TICKS(2000));
            menu_state = MENU_MAIN;
            lcd_render_menu();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ------------------ MAIN ------------------ */
void app_main(void) {
    ESP_LOGI(TAG, "Starting");

    i2c_init_bus();
    lcd_add_device();
    rgb_add_device_and_init();
    lcd_init_display();

    lcd_write_line(0, "Menu Ready");
    lcd_write_line(1, "Use Joystick");
    vTaskDelay(pdMS_TO_TICKS(1000));

    xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 5, NULL);
}
