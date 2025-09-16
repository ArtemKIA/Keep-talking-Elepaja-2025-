#ifndef LCD1602_H
#define LCD1602_H

// НЕ РАБОТАЕТ !!!!

// НЕ РАБОТАЕТ !!!!
// НЕ РАБОТАЕТ !!!!
// НЕ РАБОТАЕТ !!!!
// НЕ РАБОТАЕТ !!!!
// НЕ РАБОТАЕТ !!!!
// НЕ РАБОТАЕТ !!!!


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/gpio.h"


/* ------------------ I2C / LCD ------------------ */
 i2c_master_bus_handle_t i2c_bus = NULL;
 i2c_master_dev_handle_t lcd_dev_handle = NULL;
 i2c_master_dev_handle_t rgb_dev_handle = NULL;



esp_err_t i2c_init_bus(void) {
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

esp_err_t lcd_add_device(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = LCD_ADDRESS,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, &lcd_dev_handle);
}

esp_err_t rgb_add_device_and_init(void) {
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

/* ------------------ LCD helpers ------------------ */

esp_err_t lcd_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    esp_err_t r = i2c_master_transmit(lcd_dev_handle, buf, 2, pdMS_TO_TICKS(100));
    if (r == ESP_OK) {
        if (cmd == 0x01 || cmd == 0x02) vTaskDelay(pdMS_TO_TICKS(5));
        else vTaskDelay(pdMS_TO_TICKS(2));
    }
    return r;
}

esp_err_t lcd_data(uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    return i2c_master_transmit(lcd_dev_handle, buf, 2, pdMS_TO_TICKS(100));
}

void lcd_write_str(const char *str) {
    while (*str) {
        lcd_data((uint8_t)*str++);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/* New: write full padded line */


static void lcd_write_line(uint8_t row, const char *text) {
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16s", text); // left align, pad spaces
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

 void lcd_render_menu(void) {
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




#endif
