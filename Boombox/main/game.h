#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"

void game_init(i2c_master_bus_handle_t mcp_bus);  // MCP23017 + 7seg + buzzer
void game_start(int total_seconds);               // reset timer + MCP state
void game_update(void);                           // timer tick + 7seg + Button Game

bool game_is_won(void);                           // Button Game sequence completed
bool game_is_time_up(void);                       // timer expired
void game_stop(void);
bool button_mistake_done(void);
void game_update_mistake_leds(int mistakes);

void game_beep_mistake(void);
void game_beep_win(void);
void game_beep_fail(void);

/* NEW: enable the button game only when we actually want to play it */
void game_enable_buttons(void);