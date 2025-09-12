#ifndef SSD1306_H

#define SSD130_H

#include <stdio.h>
#include <stdint.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES (OLED_HEIGHT / 8)

#define SCREENADDRESS 0x3C

static const uint8_t init_cmds[] = {
    0x00, // control byte for commands

    0xAE, // Display OFF

    0xA8, // Set multiplex ratio
    0x3F, // 1/64 duty

    0xD3, // Display offset
    0x00, // No offset

    0x40, // Start line = 0

    // 0x20, // Memory addressing mode
    // 0x00, // Horizontal addressing

    0xA1, // Segment remap (mirror horizontally)

    0xC8, // COM scan direction (mirror vertically)

    0xDA, // Set COM pins
    0x12,

    0x81, // Contrast
    0x7F, // Values of contrast form 0-256 HEX 0x00-0xFF

    0xA4, // Set Entire Display On

    0xA6, // Set norma Display

    0xD5, // Set display Osc Frequency
    0x80, // Suggested value in Hz ration from 0-128

    0x8D, // Charge pump
    0x14, // Enable charge pump

    0xAF, // Display On
};


static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];
#endif

