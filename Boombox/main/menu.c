#include "menu.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "MENU";

/* I2C addresses */
#define LCD_ADDRESS 0x3E
#define RGB_ADDRESS 0x60
#define I2C_MASTER_FREQ_HZ (50*1000)

/* Buttons (active low, pull-up) */
#define BUTTON_UP    GPIO_NUM_5
#define BUTTON_SEL   GPIO_NUM_6
#define BUTTON_DOWN  GPIO_NUM_7

typedef enum {
    MENU_STATE_MAIN,
    MENU_STATE_DIFFICULTY,
    MENU_STATE_TIME,
    MENU_STATE_OPTION5
} menu_state_t;

#define MENU_ITEMS 5
static const char *s_menu_items[MENU_ITEMS] = {
    "Play", "Difficulty", "Time", "Option 5", "Exit"
};

static const char *s_diff_labels[3] = {"Easy", "Medium", "Hard"};

/* Internal state */
static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_lcd = NULL;
static i2c_master_dev_handle_t s_rgb = NULL;

static menu_state_t s_state = MENU_STATE_MAIN;
static int s_current_index = 0;
static int s_top_index     = 0;

static int  s_game_time    = 60;   // seconds
static int  s_difficulty    = 0;   // 0..2

static bool s_running            = false;
static bool s_req_start_game     = false;

/* ========== Low-level LCD helpers ========== */
static esp_err_t lcd_add_device(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = LCD_ADDRESS,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &dev_cfg, &s_lcd);
}

static esp_err_t rgb_add_device_and_init(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = RGB_ADDRESS,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t r = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_rgb);
    if (r != ESP_OK) return r;

    uint8_t init_data[][2] = {
        {0x80, 0x01},{0x81, 0x14},{0x82, 0xFF},
        {0x83, 0xFF},{0x84, 0xFF},{0x85, 0x20}
    };
    for (size_t i = 0; i < sizeof(init_data)/sizeof(init_data[0]); i++) {
        i2c_master_transmit(s_rgb, init_data[i], 2, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK;
}

static esp_err_t lcd_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    esp_err_t r = i2c_master_transmit(s_lcd, buf, 2, pdMS_TO_TICKS(100));
    if (r == ESP_OK) {
        if (cmd == 0x01 || cmd == 0x02)
            vTaskDelay(pdMS_TO_TICKS(5));
        else
            vTaskDelay(pdMS_TO_TICKS(2));
    }
    return r;
}

static esp_err_t lcd_data(uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    return i2c_master_transmit(s_lcd, buf, 2, pdMS_TO_TICKS(100));
}

static void lcd_write_str(const char *str) {
    while (*str) {
        lcd_data((uint8_t)*str++);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void lcd_write_line_internal(uint8_t row, const char *text) {
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16s", text);
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

/* ========== Buttons ========== */

typedef enum {
    EV_SCROLL_UP,
    EV_SCROLL_DOWN,
    EV_NONE,
    EV_PRESS
} btn_event_t;

static void buttons_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BUTTON_UP) |
                        (1ULL << BUTTON_SEL) |
                        (1ULL << BUTTON_DOWN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

static btn_event_t buttons_read_event(void) {
    static int last_up = 1, last_sel = 1, last_down = 1;
    int up   = gpio_get_level(BUTTON_UP);
    int sel  = gpio_get_level(BUTTON_SEL);
    int down = gpio_get_level(BUTTON_DOWN);

    if (up == 0 || sel == 0 || down == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
        up   = gpio_get_level(BUTTON_UP);
        sel  = gpio_get_level(BUTTON_SEL);
        down = gpio_get_level(BUTTON_DOWN);
    }

    if (sel == 0 && last_sel == 1) { last_sel = 0; return EV_PRESS; }
    if (up  == 0 && last_up  == 1) { last_up  = 0; return EV_SCROLL_UP; }
    if (down== 0 && last_down== 1){ last_down= 0; return EV_SCROLL_DOWN; }

    if (sel == 1 && last_sel == 0)   last_sel  = 1;
    if (up  == 1 && last_up  == 0)   last_up   = 1;
    if (down== 1 && last_down== 0)   last_down = 1;

    return EV_NONE;
}

/* ========== Menu drawing ========== */

static void menu_draw_list(void) {
    char line0[17], line1[17];

    snprintf(line0, sizeof(line0), "%c%s",
             (s_top_index == s_current_index) ? '>' : ' ',
             s_menu_items[s_top_index]);
    lcd_write_line_internal(0, line0);

    if (s_top_index + 1 < MENU_ITEMS) {
        snprintf(line1, sizeof(line1), "%c%s",
                 (s_top_index + 1 == s_current_index) ? '>' : ' ',
                 s_menu_items[s_top_index + 1]);
        lcd_write_line_internal(1, line1);
    } else {
        lcd_write_line_internal(1, "");
    }
}

/* ========== Public API ========== */

void menu_init(i2c_master_bus_handle_t bus) {
    s_bus = bus;
    ESP_ERROR_CHECK(lcd_add_device());
    ESP_ERROR_CHECK(rgb_add_device_and_init());
    ESP_ERROR_CHECK(lcd_init_display());
    buttons_init();
 
    s_state = MENU_STATE_MAIN;
    s_current_index = 0;
    s_top_index = 0;
    s_game_time = 60;
    s_difficulty = 0;
    s_req_start_game = false;
    s_running = false;
}

void menu_enter(void) {
    s_state = MENU_STATE_MAIN;
    s_current_index = 0;
    s_top_index = 0;
    s_req_start_game = false;
    s_running = true;
    menu_draw_list();
}

void menu_leave(void) {
    s_running = false;
}

void menu_update(void) {
    if (!s_running) return;

    btn_event_t ev = buttons_read_event();

    switch (s_state) {
    case MENU_STATE_MAIN:
        if (ev == EV_SCROLL_UP && s_current_index > 0) {
            s_current_index--;
            if (s_current_index < s_top_index) s_top_index--;
            menu_draw_list();
        } else if (ev == EV_SCROLL_DOWN && s_current_index < MENU_ITEMS - 1) {
            s_current_index++;
            if (s_current_index > s_top_index + 1) s_top_index++;
            menu_draw_list();
        } else if (ev == EV_PRESS) {
            if (s_current_index == 0) {
                // Play
                s_req_start_game = true;
                s_running = false;  // main will switch to game
            } else if (s_current_index == 1) {
                s_state = MENU_STATE_DIFFICULTY;
            } else if (s_current_index == 2) {
                s_state = MENU_STATE_TIME;
            } else if (s_current_index == 3) {
                s_state = MENU_STATE_OPTION5;
            } else if (s_current_index == 4) {
                lcd_write_line_internal(0, "Goodbye!");
                lcd_write_line_internal(1, "");
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }
        break;

    case MENU_STATE_DIFFICULTY:
        if (ev == EV_SCROLL_UP && s_difficulty < 2)      s_difficulty++;
        else if (ev == EV_SCROLL_DOWN && s_difficulty>0) s_difficulty--;
        else if (ev == EV_PRESS) {
            s_state = MENU_STATE_MAIN;
            menu_draw_list();
            break;
        }
        lcd_write_line_internal(0, "Difficulty:");
        lcd_write_line_internal(1, s_diff_labels[s_difficulty]);
        break;

    case MENU_STATE_TIME:
        if (ev == EV_SCROLL_UP && s_game_time < 60*300) s_game_time += 30;
        else if (ev == EV_SCROLL_DOWN && s_game_time > 30) s_game_time -= 30;
        else if (ev == EV_PRESS) {
            s_state = MENU_STATE_MAIN;
            menu_draw_list();
            break;
        } else {
            char buf[16];
            int minutes = s_game_time / 60;
            int secs    = s_game_time % 60;
            snprintf(buf, sizeof(buf), "%d:%02d", minutes, secs);
            lcd_write_line_internal(0, "Set Time:");
            lcd_write_line_internal(1, buf);
        }
        break;

    case MENU_STATE_OPTION5:
        lcd_write_line_internal(0, "Option 5 TBD");
        lcd_write_line_internal(1, "");
        vTaskDelay(pdMS_TO_TICKS(1000));
        s_state = MENU_STATE_MAIN;
        menu_draw_list();
        break;

    default:
        s_state = MENU_STATE_MAIN;
        menu_draw_list();
        break;
    }
}

bool menu_request_start_game(void) {
    bool r = s_req_start_game;
    s_req_start_game = false;
    return r;
}

int menu_get_game_time(void) {
    return s_game_time;
}

int menu_get_difficulty(void) {
    return s_difficulty;
}

void menu_show_status(const char *line0, const char *line1) {
    lcd_write_line_internal(0, line0);
    lcd_write_line_internal(1, line1);
}