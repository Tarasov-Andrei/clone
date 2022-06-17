#ifndef BMP180_H
#define BMP180_H

#include <stdio.h>
#include "stdint.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

/**
 * @brief Инициализация устройства
 * @param i2c_port Номер порта I2C. (I2C_NUM_0 или I2C_NUM_1)
 */
void bmp180_init(i2c_port_t i2c_port);

/**
 * @brief Считать температуру
 * @return Температура в *C
 */
float bmp180_read_temp(void);

/**
 * @brief Считать давление
 * @return Давление в mmHg
 */
uint16_t bmp180_read_press(void);

#endif // BMP180_H
