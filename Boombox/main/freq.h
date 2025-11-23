// freq.h
#pragma once

#include <stdbool.h>

void freq_init(void);                          // init SPI OLED + ADC for K pot
void freq_start(void);                         // reset state, start drawing
void freq_update(void);                        // read pot + redraw frame
void freq_stop(void);

bool freq_is_matched(void);                    // k close enough to target
void freq_start_task(void);