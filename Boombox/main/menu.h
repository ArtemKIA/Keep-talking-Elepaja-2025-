#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"

void menu_init(i2c_master_bus_handle_t bus);   // LCD + RGB on bus0
void menu_enter(void);                         // enter main menu
void menu_update(void);                        // handle buttons + redraw menu

bool menu_request_start_game(void);            // latched start request
int  menu_get_game_time(void);                 // seconds
int  menu_get_difficulty(void);                // 0..2

void menu_show_status(const char *line0, const char *line1);
