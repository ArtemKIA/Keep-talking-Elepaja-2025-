// main.c (combined version)
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

/* ========== PIN CONFIG - change if needed ========== */
/* I2C / LCD */
#define I2C_BUS_PORT 0
#define PIN_NUM_SDA 13
#define PIN_NUM_SCL 14
#define LCD_ADDRESS 0x3E
#define RGB_ADDRESS 0x60
#define I2C_MASTER_FREQ_HZ (50 * 1000)

/* I2C for MCP23017 GPIO Expander */
#define I2C_MCP23017_PORT 1
#define MCP23017_SDA_IO 48
#define MCP23017_SCL_IO 47
#define MCP23017_ADDR 0x20

/* MCP23017 Registers */
#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA  0x12
#define GPIOB  0x13
#define OLATA  0x14
#define OLATB  0x15
#define GPPUA  0x0C  // Pull-up resistor register

/* MCP23017 Pins */
#define LEDS_MASK 0x07 // GPA0, GPA1, GPA2
#define BUTTON_PIN 0x08 // GPA3

/* Buttons (active low, internal pull-ups) */
#define BUTTON_UP   GPIO_NUM_5
#define BUTTON_SEL  GPIO_NUM_6
#define BUTTON_DOWN GPIO_NUM_7

/* Shift register (7-seg) pins */
#define LATCH_PIN  GPIO_NUM_9
#define DATA_PIN   GPIO_NUM_18
#define CLOCK_PIN  GPIO_NUM_10

/* Extra outputs */
#define LED_PIN    GPIO_NUM_45
#define BUZZER_PIN GPIO_NUM_4

/* ========== OTHER CONFIG ========== */
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_11

static const char *TAG = "MAIN";

/* ========== MENU ========== */
static int game_time = 60; // seconds default
static int difficulty = 0;
const char *difficulty_labels[3] = {"Easy", "Medium", "Hard"};
typedef enum { MENU_MAIN, MENU_PLAY, MENU_DIFFICULTY, MENU_TIME, MENU_OPTION5 } menu_state_t;
static menu_state_t menu_state = MENU_MAIN;
static int current_index = 0;
static int top_index = 0;
#define MENU_ITEMS 5
const char *menu[MENU_ITEMS] = { "Play", "Difficulty", "Time", "Option 5", "Exit" };

typedef enum { SCROLL_UP, SCROLL_DOWN, NO_SCROLL, PRESS } joystick_event_t;

/* ========== I2C/LCD handles ========== */
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_bus_handle_t i2c_mcp23017_bus = NULL;
static i2c_master_dev_handle_t lcd_dev_handle = NULL;
static i2c_master_dev_handle_t rgb_dev_handle = NULL;
static i2c_master_dev_handle_t mcp23017_dev_handle = NULL;

/* ========== Timer / 7-seg variables ========== */
static unsigned int timerSeconds = 10 * 60 + 5;
static int64_t lastUpdate = 0;
static unsigned int lastMinutes = 0;
static volatile bool game_running = false;
static TaskHandle_t timer_task_handle = NULL;
static TaskHandle_t mcp23017_task_handle = NULL;

/* Game state variables */
static volatile uint8_t led_count = 0;
static volatile bool game_won = false;

/* 7-seg maps */
static const uint8_t digit_map[4] = { 0b00001110, 0b00001101, 0b00001011, 0b00000111 };
static const uint8_t segment_map[10] = {
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

/* ========== HELPERS ========== */
void format_time(int seconds, char *buf, size_t len) {
    int minutes = seconds / 60;
    int secs = seconds % 60;
    snprintf(buf, len, "%d:%02d", minutes, secs);
}

/* ========== I2C + LCD helpers ========== */
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

static esp_err_t i2c_init_mcp23017_bus(void) {
    if (i2c_mcp23017_bus) return ESP_OK;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MCP23017_PORT,
        .sda_io_num = MCP23017_SDA_IO,
        .scl_io_num = MCP23017_SCL_IO,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &i2c_mcp23017_bus);
}

static esp_err_t lcd_add_device(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = LCD_ADDRESS,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, &lcd_dev_handle);
}

static esp_err_t mcp23017_add_device(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = MCP23017_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_mcp23017_bus, &dev_cfg, &mcp23017_dev_handle);
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

static void lcd_write_line(uint8_t row, const char *text) {
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

/* ========== MCP23017 Functions ========== */
static esp_err_t mcp23017_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(mcp23017_dev_handle, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t mcp23017_read_reg(uint8_t reg, uint8_t *val) {
    esp_err_t ret = i2c_master_transmit_receive(mcp23017_dev_handle, &reg, 1, val, 1, pdMS_TO_TICKS(100));
    return ret;
}

static esp_err_t mcp23017_init(void) {
    // Set GPA0, GPA1, GPA2 as outputs (LEDS), GPA3 as input (BUTTON), rest as input
    esp_err_t ret = mcp23017_write_reg(IODIRA, 0xF8); // 0b11111000 (GPA0-2 output, GPA3-7 input)
    if (ret != ESP_OK) return ret;
    
    // Enable pull-up resistor on button pin (GPA3)
    ret = mcp23017_write_reg(GPPUA, 0x08);  // 0b00001000 - enable pull-up on GPA3
    if (ret != ESP_OK) return ret;
    
    // Turn all LEDs off initially
    ret = mcp23017_write_reg(GPIOA, 0x00);
    
    ESP_LOGI(TAG, "MCP23017 initialized");
    return ret;
}

void mcp23017_task(void *pvParameters) {
    uint8_t last_button_state = 1; // Start with button not pressed (pull-up)
    uint8_t current_button_state;
    uint8_t read_val;
    
    // Reset LED count at game start
    led_count = 0;
    game_won = false;
    
    ESP_LOGI(TAG, "MCP23017 task started");
    
    while (game_running && !game_won) {
        // Read GPIOA
        if (mcp23017_read_reg(GPIOA, &read_val) == ESP_OK) {
            // Button is active LOW (pressed = 0, not pressed = 1 due to pull-up)
            current_button_state = (read_val & BUTTON_PIN) ? 0 : 1;
            
            // Check for button press (rising edge detection)
            if (current_button_state == 1 && last_button_state == 0) {
                // Button was just pressed - increment counter
                led_count++;
                if (led_count > 7) led_count = 0; // Wrap around after 7
                
                // Update LEDs with binary representation of count
                mcp23017_write_reg(GPIOA, (read_val & ~LEDS_MASK) | (led_count & LEDS_MASK));
                
                // Check if player won (LEDs show 111 = binary 7)
                if (led_count == 7) {
                    game_won = true;
                    ESP_LOGI(TAG, "Player won! LEDs reached 111");
                }
                
                // Small delay to debounce
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            
            last_button_state = current_button_state;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay for debouncing
    }
    
    ESP_LOGI(TAG, "MCP23017 task ended");
    vTaskDelete(NULL);
}

/* Renders two menu lines based on top_index/current_index */
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

/* ========== BUTTONS ========== */
void joystick_init(void) {
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_UP) | (1ULL << BUTTON_SEL) | (1ULL << BUTTON_DOWN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button_config);
    ESP_LOGI(TAG, "Buttons initialised (UP=%d SEL=%d DOWN=%d)", BUTTON_UP, BUTTON_SEL, BUTTON_DOWN);
}

/* Debounced edge detection. Priority: SELECT > UP > DOWN */
joystick_event_t joystick_read_event(void) {
    static int last_up = 1, last_sel = 1, last_down = 1;
    int up = gpio_get_level(BUTTON_UP);
    int sel = gpio_get_level(BUTTON_SEL);
    int down = gpio_get_level(BUTTON_DOWN);

    if (up == 0 || sel == 0 || down == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
        up  = gpio_get_level(BUTTON_UP);
        sel = gpio_get_level(BUTTON_SEL);
        down= gpio_get_level(BUTTON_DOWN);
    }

    if (sel == 0 && last_sel == 1) { last_sel = 0; return PRESS; }
    if (up == 0 && last_up == 1)   { last_up  = 0; return SCROLL_UP; }
    if (down == 0 && last_down == 1){ last_down= 0; return SCROLL_DOWN; }

    if (sel == 1 && last_sel == 0) last_sel = 1;
    if (up == 1 && last_up == 0)   last_up = 1;
    if (down == 1 && last_down == 0) last_down = 1;

    return NO_SCROLL;
}

/* ========== SHIFT OUT / 7-SEG ========== */
void shiftOut(int dataPin, int clockPin, int bitOrder, uint8_t val) {
    for (int i = 0; i < 8; i++) {
        int bit;
        if (bitOrder == 1) bit = !!(val & (1 << (7 - i))); // MSBFIRST
        else bit = !!(val & (1 << i));
        gpio_set_level(dataPin, bit);
        gpio_set_level(clockPin, 1);
        ets_delay_us(1);
        gpio_set_level(clockPin, 0);
    }
}

void showDigit(int pos, int num, bool withDot) {
    gpio_set_level(LATCH_PIN, 0);
    uint8_t seg = segment_map[num];
    if (withDot) seg |= DP;
    shiftOut(DATA_PIN, CLOCK_PIN, 1, digit_map[pos]);
    shiftOut(DATA_PIN, CLOCK_PIN, 1, seg);
    gpio_set_level(LATCH_PIN, 1);
    ets_delay_us(1000);
}

void displayTime7seg() {
    int minutes = timerSeconds / 60;
    int seconds = timerSeconds % 60;
    int d1 = minutes / 10;
    int d2 = minutes % 10;
    int d3 = seconds / 10;
    int d4 = seconds % 10;

    showDigit(0, d1, false);
    showDigit(1, d2, true);
    showDigit(2, d3, false);
    showDigit(3, d4, false);
}

/* ========== Buzzer/LED helpers ========== */
void buzzer_on() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void buzzer_off() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void beepBuzzer(int times) {
    for (int i = 0; i < times; i++) {
        gpio_set_level(LED_PIN, 1);
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_PIN, 0);
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void victory_sound(void) {
    for (int i = 0; i < 5; i++) {
        gpio_set_level(LED_PIN, 1);
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_PIN, 0);
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ========== Timer logic ========== */
void onMinutePassed(void) {
    ESP_LOGI(TAG, "Minute passed! Timer = %u seconds", timerSeconds);
    beepBuzzer(3);
}

void updateTimerTick() {
    int64_t now = esp_timer_get_time();
    if ((now - lastUpdate) >= 1000000 && timerSeconds > 0) {
        timerSeconds--;
        lastUpdate = now;
        unsigned int currentMinutes = timerSeconds / 60;
        if (currentMinutes != lastMinutes) {
            lastMinutes = currentMinutes;
            onMinutePassed();
        }
    }
}

/* ========== timer_task ========== */
void timer_task(void *pv) {
    ESP_LOGI(TAG, "Timer task started");

    while (game_running) {
        updateTimerTick();
        displayTime7seg();

        // Check if player won the game
        if (game_won) {
            ESP_LOGI(TAG, "Player won! Stopping game");
            game_running = false;
            
            // Show victory message on LCD
            lcd_write_line(0, "You Win!");
            lcd_write_line(1, "Congratulations!");
            
            // Play victory sound
            victory_sound();
            vTaskDelay(pdMS_TO_TICKS(2000));
            break;
        }

        if (timerSeconds == 0) {
            // Time's up
            for (int i = 0; i < 3; ++i) {
                gpio_set_level(LED_PIN, 1);
                buzzer_on();
                vTaskDelay(pdMS_TO_TICKS(300));
                gpio_set_level(LED_PIN, 0);
                buzzer_off();
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            game_running = false;
            
            // Show game over message
            lcd_write_line(0, "Time's up!");
            lcd_write_line(1, "Game Over");
            vTaskDelay(pdMS_TO_TICKS(2000));
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Clean up tasks
    if (mcp23017_task_handle != NULL) {
        vTaskDelete(mcp23017_task_handle);
        mcp23017_task_handle = NULL;
    }

    timer_task_handle = NULL;
    menu_state = MENU_MAIN;
    lcd_render_menu();
    vTaskDelete(NULL);
}

/* ========== Start / Stop game helpers ========== */
void stop_game(void) {
    if (!game_running) return;
    game_running = false;
    buzzer_off();
    ESP_LOGI(TAG, "Game stopped by user");
    
    // Clean up tasks
    if (mcp23017_task_handle != NULL) {
        vTaskDelete(mcp23017_task_handle);
        mcp23017_task_handle = NULL;
    }
}

void start_game(void) {
    if (game_running) return;

    timerSeconds = game_time;
    lastUpdate = esp_timer_get_time();
    lastMinutes = timerSeconds / 60;
    game_running = true;
    game_won = false;

    // Initialize MCP23017 for the game
    if (mcp23017_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCP23017");
        lcd_write_line(0, "GPIO Expander");
        lcd_write_line(1, "Init Failed!");
        vTaskDelay(pdMS_TO_TICKS(2000));
        game_running = false;
        menu_state = MENU_MAIN;
        lcd_render_menu();
        return;
    }

    // Show LCD message
    lcd_write_line(0, "Game Running");
    lcd_write_line(1, "Press Button!");

    // Create timer task and MCP23017 task
    xTaskCreate(timer_task, "timer_task", 4096, NULL, 5, &timer_task_handle);
    xTaskCreate(mcp23017_task, "mcp23017_task", 4096, NULL, 4, &mcp23017_task_handle);
}

/* ========== Menu task ========== */
void joystick_task(void *pvParameters) {
    joystick_init();
    lcd_render_menu();

    while (1) {
        joystick_event_t event = joystick_read_event();

        if (game_running) {
            /* When timer running: allow SELECT to stop and return to menu */
            if (event == PRESS) {
                stop_game();
                vTaskDelay(pdMS_TO_TICKS(200)); /* debounce */
            }
            /* while running, ignore other menu inputs */
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

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
                if (current_index == 0) {
                    /* Start game */
                    lcd_write_line(0, "Game Starting...");
                    lcd_write_line(1, "");
                    vTaskDelay(pdMS_TO_TICKS(500));
                    start_game();
                } else if (current_index == 1) menu_state = MENU_DIFFICULTY;
                else if (current_index == 2) menu_state = MENU_TIME;
                else if (current_index == 3) menu_state = MENU_OPTION5;
                else if (current_index == 4) {
                    lcd_write_line(0, "Goodbye!");
                    lcd_write_line(1, "");
                    vTaskDelay(pdMS_TO_TICKS(1500));
                }
            }
            break;

        case MENU_PLAY:
            /* If we get here while not running, go back to MAIN */
            lcd_write_line(0, "Playing...");
            lcd_write_line(1, "");
            vTaskDelay(pdMS_TO_TICKS(200));
            break;

        case MENU_DIFFICULTY:
            if (event == SCROLL_UP && difficulty < 2) difficulty++;
            else if (event == SCROLL_DOWN && difficulty > 0) difficulty--;
            else if (event == PRESS) { menu_state = MENU_MAIN; lcd_render_menu(); break; }

            lcd_write_line(0, "Difficulty:");
            lcd_write_line(1, difficulty_labels[difficulty]);
            break;

        case MENU_TIME:
            if (event == SCROLL_UP && game_time < 60*300) game_time += 30;
            else if (event == SCROLL_DOWN && game_time > 30) game_time -= 30;
            else if (event == PRESS) { menu_state = MENU_MAIN; lcd_render_menu(); break; }

            {
                char buf[16];
                format_time(game_time, buf, sizeof(buf));
                lcd_write_line(0, "Set Time:");
                lcd_write_line(1, buf);
            }
            break;

        case MENU_OPTION5:
            lcd_write_line(0, "Option 5 TBD");
            lcd_write_line(1, "");
            vTaskDelay(pdMS_TO_TICKS(1000));
            menu_state = MENU_MAIN;
            lcd_render_menu();
            break;

        default:
            menu_state = MENU_MAIN;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ========== app_main (initialisation) ========== */
void app_main(void) {
    ESP_LOGI(TAG, "Starting combined menu+timer+GPIO expander");

    /* I2C / LCD */
    i2c_init_bus();
    lcd_add_device();
    rgb_add_device_and_init();
    lcd_init_display();

    /* I2C for MCP23017 */
    i2c_init_mcp23017_bus();
    mcp23017_add_device();

    /* Setup shift-register + LED pins (outputs) */
    gpio_config_t io_conf_out = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LATCH_PIN) | (1ULL << DATA_PIN) | (1ULL << CLOCK_PIN) | (1ULL << LED_PIN)
    };
    gpio_config(&io_conf_out);

    /* Setup buzzer PWM (LEDC) */
    ledc_timer_config_t buzzer_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 2000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&buzzer_timer);

    ledc_channel_config_t buzzer_channel = {
        .gpio_num       = BUZZER_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&buzzer_channel);

    /* init buttons and menu task */
    joystick_init();

    lcd_write_line(0, "System Ready");
    lcd_write_line(1, "Use Buttons");
    vTaskDelay(pdMS_TO_TICKS(1000));

    xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 5, NULL);
}
