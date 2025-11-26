#include "menu.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "rom/ets_sys.h"

static const char *TAG = "MENU";

#define I2C_MASTER_FREQ_HZ   (50 * 1000)

/* PCF8574 backpack mapping:
 * P0 -> RS
 * P1 -> RW
 * P2 -> EN
 * P3 -> Backlight
 * P4 -> D4
 * P5 -> D5
 * P6 -> D6
 * P7 -> D7
 */
#define LCD_ADDR_DEFAULT 0x27

#define LCD_PIN_RS  0x01
#define LCD_PIN_RW  0x02
#define LCD_PIN_EN  0x04
#define LCD_PIN_BL  0x08

/* front-panel buttons (active low, pull-ups) */
#define BUTTON_UP    GPIO_NUM_5
#define BUTTON_SEL   GPIO_NUM_6
#define BUTTON_DOWN  GPIO_NUM_7

typedef enum {
    EV_SCROLL_UP,
    EV_SCROLL_DOWN,
    EV_NONE,
    EV_PRESS
} btn_event_t;

typedef enum {
    MENU_STATE_MAIN,
    MENU_STATE_DIFFICULTY,
    MENU_STATE_TIME,
    MENU_STATE_OPTION5
} menu_state_t;

/* top-level menu strings (two lines shown at a time) */
#define MENU_ITEMS 5
static const char *s_menu_items[MENU_ITEMS] = {
    "Play",
    "Difficulty",
    "Time",
    "Option 5",
    "Exit"
};

/* difficulty labels: purely cosmetic for now */
static const char *s_diff_labels[3] = {
    "Easy",
    "Medium",
    "Hard"
};

/* I2C + LCD state */
static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_lcd = NULL;
static uint8_t s_backlight_mask = LCD_PIN_BL;

/* menu navigation state */
static menu_state_t s_state = MENU_STATE_MAIN;
static int  s_current_index   = 0;   // currently selected item
static int  s_top_index       = 0;   // first visible line

/* configuration managed by menu */
static int  s_game_time       = 180;  // default 60s
static int  s_difficulty      = 0;   // 0..2 for display only
static bool s_running         = false;
static bool s_req_start_game  = false;

static bool s_lcd_ok = false;

/* ========== low-level LCD helpers (PCF8574 + HD44780) ========== */

static esp_err_t lcd_write_raw(uint8_t v)
{
    if (!s_lcd_ok || s_lcd == NULL) {
        return ESP_FAIL;
    }
    return i2c_master_transmit(s_lcd, &v, 1, -1);
}

/* send single 4-bit nibble with RS flag already encoded */
static esp_err_t lcd_send_nibble(uint8_t nibble, uint8_t rs_flag)
{
    if (!s_lcd_ok || s_lcd == NULL) {
        return ESP_FAIL;
    }

    nibble &= 0x0F;

    uint8_t out = 0;

    /* move D4..D7 into P4..P7 */
    out |= (nibble << 4);
    out |= s_backlight_mask;

    if (rs_flag) {
        out |= LCD_PIN_RS;
    }

    /* always write-only */
    out &= (uint8_t)~LCD_PIN_RW;

    esp_err_t err;

    /* EN high */
    err = lcd_write_raw(out | LCD_PIN_EN);
    if (err != ESP_OK) return err;
    ets_delay_us(600);

    /* EN low, data latched */
    err = lcd_write_raw(out & (uint8_t)~LCD_PIN_EN);
    if (err != ESP_OK) return err;
    ets_delay_us(600);

    return ESP_OK;
}

/* send full byte as command or data */
static esp_err_t lcd_send_byte(uint8_t value, bool is_data)
{
    esp_err_t err;

    err = lcd_send_nibble((value >> 4) & 0x0F, is_data);
    if (err != ESP_OK) return err;

    err = lcd_send_nibble(value & 0x0F, is_data);
    if (err != ESP_OK) return err;

    ets_delay_us(50);
    return ESP_OK;
}

/* convenience wrappers for commands / data */
static inline esp_err_t lcd_cmd(uint8_t cmd)
{
    esp_err_t err = lcd_send_byte(cmd, false);
    if (err != ESP_OK) {
        return err;
    }
    /* clear / home need longer delay */
    if (cmd == 0x01 || cmd == 0x02) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_OK;
}

static inline esp_err_t lcd_data(uint8_t data)
{
    return lcd_send_byte(data, true);
}

/* write zero-terminated ASCII string */
static void lcd_write_str(const char *s)
{
    if (!s_lcd_ok) return;
    while (*s) {
        lcd_data((uint8_t)*s++);
    }
}

/* write 16-char line, padded with spaces, row 0 or 1 */
static void lcd_write_line_internal(uint8_t row, const char *text)
{
    if (!s_lcd_ok) return;

    char buf[17];
    if (text) {
        snprintf(buf, sizeof(buf), "%-16s", text);
    } else {
        snprintf(buf, sizeof(buf), "%-16s", "");
    }

    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    lcd_cmd(0x80 | addr);
    lcd_write_str(buf);
}

/* HD44780 initialisation sequence for 4-bit mode */
static esp_err_t lcd_init_display(void)
{
    if (s_lcd == NULL) {
        ESP_LOGE(TAG, "lcd_init_display called with NULL device");
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // power-on delay

    esp_err_t err;

    uint8_t base = s_backlight_mask;
    err = lcd_write_raw(base);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 4-bit init “magic” sequence */
    err = lcd_send_nibble(0x03, false);
    if (err != ESP_OK) return err;
    ets_delay_us(4500);

    err = lcd_send_nibble(0x03, false);
    if (err != ESP_OK) return err;
    ets_delay_us(4500);

    err = lcd_send_nibble(0x03, false);
    if (err != ESP_OK) return err;
    ets_delay_us(150);

    err = lcd_send_nibble(0x02, false);
    if (err != ESP_OK) return err;
    ets_delay_us(150);

    /* 2-line, 5x8 dots */
    ESP_ERROR_CHECK_WITHOUT_ABORT(lcd_cmd(0x28));
    /* display on, cursor off */
    ESP_ERROR_CHECK_WITHOUT_ABORT(lcd_cmd(0x0C));
    /* clear display */
    ESP_ERROR_CHECK_WITHOUT_ABORT(lcd_cmd(0x01));
    vTaskDelay(pdMS_TO_TICKS(2));
    /* entry mode: increment, no shift */
    ESP_ERROR_CHECK_WITHOUT_ABORT(lcd_cmd(0x06));

    ESP_LOGI(TAG, "LCD initialised (1602 I2C backpack, PCF8574)");
    return ESP_OK;
}

/* attach PCF8574 device and probe LCD */
static esp_err_t lcd_add_device(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = LCD_ADDR_DEFAULT,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(s_bus, &cfg, &s_lcd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add LCD device: %s", esp_err_to_name(err));
        return err;
    }

    /* one-byte write to verify presence */
    uint8_t test = s_backlight_mask;
    err = i2c_master_transmit(s_lcd, &test, 1, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD probe failed at 0x%02X: %s",
                 LCD_ADDR_DEFAULT, esp_err_to_name(err));
        s_lcd_ok = false;
        return err;
    }

    ESP_LOGI(TAG, "LCD: device added at 0x%02X", LCD_ADDR_DEFAULT);
    s_lcd_ok = true;
    return ESP_OK;
}

/* ========== button handling ========== */

static void buttons_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BUTTON_UP) |
                        (1ULL << BUTTON_SEL) |
                        (1ULL << BUTTON_DOWN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
}

/* simple edge-based event decoding */
static btn_event_t buttons_read_event(void)
{
    static int last_up = 1, last_sel = 1, last_down = 1;

    int up   = gpio_get_level(BUTTON_UP);
    int sel  = gpio_get_level(BUTTON_SEL);
    int down = gpio_get_level(BUTTON_DOWN);

    if (sel == 0 && last_sel == 1) {
        last_sel = 0;
        return EV_PRESS;
    }
    if (up == 0 && last_up == 1) {
        last_up = 0;
        return EV_SCROLL_UP;
    }
    if (down == 0 && last_down == 1) {
        last_down = 0;
        return EV_SCROLL_DOWN;
    }

    if (sel == 1)  last_sel  = 1;
    if (up == 1)   last_up   = 1;
    if (down == 1) last_down = 1;

    return EV_NONE;
}

/* render the two visible menu items on the LCD */
static void menu_draw_list(void)
{
    char line0[17];
    char line1[17];

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

/* ========== public API ========== */

void menu_init(i2c_master_bus_handle_t bus)
{
    s_bus = bus;

    esp_err_t err = lcd_add_device();
    if (err != ESP_OK) {
        s_lcd_ok = false;
        ESP_LOGE(TAG, "LCD add device failed, menu will run headless");
    } else {
        err = lcd_init_display();
        if (err != ESP_OK) {
            s_lcd_ok = false;
            ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(err));
        }
    }

    buttons_init();

    s_state = MENU_STATE_MAIN;
}

/* enter main menu screen, reset scroll position */
void menu_enter(void)
{
    s_state          = MENU_STATE_MAIN;
    s_current_index  = 0;
    s_top_index      = 0;
    s_req_start_game = false;
    s_running        = true;

    menu_draw_list();
}

void menu_leave(void)
{
    s_running = false;
}

/* call frequently from main loop while in menu state */
void menu_update(void)
{
    if (!s_running) {
        return;
    }

    btn_event_t ev = buttons_read_event();

    switch (s_state) {
    case MENU_STATE_MAIN:
        if (ev == EV_SCROLL_UP && s_current_index > 0) {
            s_current_index--;
            if (s_current_index < s_top_index) {
                s_top_index--;
            }
            menu_draw_list();
        } else if (ev == EV_SCROLL_DOWN && s_current_index < MENU_ITEMS - 1) {
            s_current_index++;
            if (s_current_index > s_top_index + 1) {
                s_top_index++;
            }
            menu_draw_list();
        } else if (ev == EV_PRESS) {
            if (s_current_index == 0) {
                /* Play */
                s_req_start_game = true;
                s_running = false;
            } else if (s_current_index == 1) {
                s_state = MENU_STATE_DIFFICULTY;
            } else if (s_current_index == 2) {
                s_state = MENU_STATE_TIME;
            } else if (s_current_index == 3) {
                s_state = MENU_STATE_OPTION5;
            } else if (s_current_index == 4) {
                /* Exit: small goodbye message */
                lcd_write_line_internal(0, "Goodbye!");
                lcd_write_line_internal(1, "");
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }
        break;

    case MENU_STATE_DIFFICULTY:
        /* difficulty selection (for now only displayed) */
        if (ev == EV_SCROLL_UP && s_difficulty < 2) {
            s_difficulty++;
        } else if (ev == EV_SCROLL_DOWN && s_difficulty > 0) {
            s_difficulty--;
        } else if (ev == EV_PRESS) {
            /* back to main menu */
            s_state = MENU_STATE_MAIN;
            menu_draw_list();
            break;
        }

        lcd_write_line_internal(0, "Difficulty:");
        lcd_write_line_internal(1, s_diff_labels[s_difficulty]);
        break;

    case MENU_STATE_TIME:
        /* game time in 30-second steps, 30 s .. 300 min */
        if (ev == EV_SCROLL_UP && s_game_time < 60 * 300) {
            s_game_time += 30;
        } else if (ev == EV_SCROLL_DOWN && s_game_time > 30) {
            s_game_time -= 30;
        } else if (ev == EV_PRESS) {
            s_state = MENU_STATE_MAIN;
            menu_draw_list();
            break;
        }

        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d:%02d",
                     s_game_time / 60,
                     s_game_time % 60);
            lcd_write_line_internal(0, "Set Time:");
            lcd_write_line_internal(1, buf);
        }
        break;

    case MENU_STATE_OPTION5:
        /* placeholder for future extension */
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

/* one-shot “start game” latch checked from main.c */
bool menu_request_start_game(void)
{
    bool r = s_req_start_game;
    s_req_start_game = false;
    return r;
}

/* selected game time in seconds */
int menu_get_game_time(void)
{
    return s_game_time;
}

/* simple 2-line status helper (used during game) */
void menu_show_status(const char *line0, const char *line1)
{
    lcd_write_line_internal(0, line0);
    lcd_write_line_internal(1, line1);
}