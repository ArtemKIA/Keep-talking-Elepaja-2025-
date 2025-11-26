#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"

// 0.96" I2C OLED (SSD1306 128x64) + 3 pots on GPIO15/16/17 (ADC2)
// P1 = amplitude, P2 = period/frequency, P3 = noise amount

void freq_init(i2c_master_bus_handle_t bus);  // pass bus1 handle
void freq_start(void);                        // reset state, start drawing
void freq_update(void);                       // read pots + redraw frame
void freq_stop(void);                         // stop updating

bool freq_is_matched(void);                   // k close enough to target
