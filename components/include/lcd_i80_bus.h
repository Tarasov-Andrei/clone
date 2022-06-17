#ifndef LCD_I80_BUS_H
#define LCD_I80_BUS_H

#include "stdint.h"
#include "sdkconfig.h"
#include "stdio.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/periph_ctrl.h"
#include "hal/gpio_hal.h"
#include "soc/periph_defs.h"
#include "hal/i2s_ll.h"
#include "esp_log.h"
#include "esp32/rom/ets_sys.h"

#define I2S_NUM 0

#if I2S_NUM
#define i2s I2S1
#else
#define i2s I2S0
#endif

/**************** LCD PIN ***************/
#define LCD_DB0 0
#define LCD_DB1 4
#define LCD_DB2 25
#define LCD_DB3 26
#define LCD_DB4 27
#define LCD_DB5 14
#define LCD_DB6 12
#define LCD_DB7 13
#define LCD_DC 2
#define LCD_WR 15
/******************************************/

/**
 * @brief Инициализация 8-бит шины
 * @param freq_pclk_Mhz Частота тактирования, МГц
 */
void lcd_i80_bus_init(uint8_t freq_pclk_Mhz);

#endif // LCD_I80_BUS_H
