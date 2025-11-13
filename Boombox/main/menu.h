#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"

void menu_init(i2c_master_bus_handle_t bus);  // LCD+RGB on bus0
void menu_enter(void);                        // show menu screen
void menu_update(void);                       // handle buttons + LCD
void menu_leave(void);                        // optional, for clearing

bool menu_request_start_game(void);           // one-shot flag when "Play" pressed
int  menu_get_game_time(void);                // seconds
int  menu_get_difficulty(void);               // 0..2

void menu_show_status(const char *line0, const char *line1); // generic status/msgs
