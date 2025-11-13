// maze.h
#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"

void maze_init(i2c_master_bus_handle_t bus);  // pass I2C bus 0
void maze_start(void);
void maze_update(void);
void maze_stop(void);

bool maze_is_finished(void);  // reached goal + button