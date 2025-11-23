#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"

void game_init(i2c_master_bus_handle_t mcp_bus);  // bus1 for MCP23017, plus 7seg + buzzer GPIOs
void game_start(int total_seconds);               // reset timer + MCP state
void game_update(void);                           // timer tick + 7seg + MCP button/LEDs

bool game_is_won(void);                           // MCP23017 LEDs pattern reached
bool game_is_time_up(void);                       // timer expired
void game_stop(void);
void game_update_mistake_leds(int mistakes);

void game_beep_mistake(void);
void game_beep_win(void);
void game_beep_fail(void);