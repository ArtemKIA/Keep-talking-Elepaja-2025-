#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "maze.h"
#include "menu.h"
#include "game.h"

static const char *TAG = "MAIN";

/* I2C bus config */
#define BUS0_PORT   0    // LCD + RGB + HT16K33 (maze)
#define BUS1_PORT   1    // MCP23017
#define SDA0_PIN    13
#define SCL0_PIN    14
#define SDA1_PIN    48
#define SCL1_PIN    47

static i2c_master_bus_handle_t s_bus0 = NULL;
static i2c_master_bus_handle_t s_bus1 = NULL;

typedef enum {
    APP_STATE_MENU,
    APP_STATE_GAME
} app_state_t;

static app_state_t s_state = APP_STATE_MENU;

static void i2c_init_buses(void) {
    i2c_master_bus_config_t bus0_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = BUS0_PORT,
        .sda_io_num = SDA0_PIN,
        .scl_io_num = SCL0_PIN,
        .flags = {.enable_internal_pullup = true},
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus0_cfg, &s_bus0));

    i2c_master_bus_config_t bus1_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = BUS1_PORT,
        .sda_io_num = SDA1_PIN,
        .scl_io_num = SCL1_PIN,
        .flags = {.enable_internal_pullup = true},
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus1_cfg, &s_bus1));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting MAZE ONLY mode");

    // Initialize both I²C buses (or only bus0 if you want)
    i2c_init_buses();

    // ONLY initialize the maze module
    maze_init(s_bus0);
    maze_start();

    // MAZE ONLY loop
    while (1) {
        maze_update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*
void app_main(void) {
    ESP_LOGI(TAG, "Starting modular app (menu + maze + game)");

    i2c_init_buses();

    // init modules
    menu_init(s_bus0);    // LCD + RGB
    maze_init(s_bus0);    // HT16K33 + joystick ADC
    game_init(s_bus1);    // MCP23017 + 7seg + buzzer

    // enter menu
    menu_enter();
    s_state = APP_STATE_MENU;

    while (1) {
        switch (s_state) {
        case APP_STATE_MENU:
            menu_update();

            if (menu_request_start_game()) {
                int game_time = menu_get_game_time();
                // int diff = menu_get_difficulty(); // We can use this later

                maze_start();
                game_start(game_time);

                menu_show_status("Maze Running", "Use Joystick");
                s_state = APP_STATE_GAME;
            }
            break;

        case APP_STATE_GAME:
            maze_update();
            game_update();

            if (maze_is_finished() || game_is_won()) {
                game_stop();
                maze_stop();
                menu_show_status("You Win!", "Congratulations!");
                vTaskDelay(pdMS_TO_TICKS(2000));

                menu_enter();
                s_state = APP_STATE_MENU;

            } else if (game_is_time_up()) {
                game_stop();
                maze_stop();
                menu_show_status("Time's up!", "Game Over");
                vTaskDelay(pdMS_TO_TICKS(2000));

                menu_enter();
                s_state = APP_STATE_MENU;
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
    */