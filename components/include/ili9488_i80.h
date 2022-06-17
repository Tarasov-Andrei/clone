#ifndef ILI9488_I80_H
#define ILI9488_I80_H

#include "stdint.h"

/******* Colors *******/
#define LCD_BLACK 0x0000
#define LCD_BLUE 0x001F
#define LCD_RED 0xF800
#define LCD_GREEN 0x07E0
#define LCD_CYAN 0x07FF
#define LCD_MAGENTA 0xF81F
#define LCD_YELLOW 0xFFE0
#define LCD_WHITE 0xFFFF
#define LCD_LIGHTGREY 0xC618

/**
 * @brief Инициализация дисплея
 * @param lcd_width Ширина дисплея
 * @param lcd_heght Высота дисплея
 */
void ili9488_i80_init(uint16_t lcd_width, uint16_t lcd_heght);

/**
 * @brief Заливка экрана
 * @param color Цвет
 */
void lcd_fill_screen(uint16_t color);

/**
 * @brief Залитый прямоугольник
 * @param x0 Начало по x
 * @param y0 Начало по y
 * @param x1 Конец по x
 * @param y1 Конец по y
 * @param color Цвет
 */
void lcd_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Прямоугольник
 * @param x0 Начало по x
 * @param y0 Начало по y
 * @param x1 Конец по x
 * @param y1 Конец по y
 * @param color Цвет
 */
void lcd_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Горизонтальная линия
 * @param x0 Начало по x
 * @param y0 Начало по y
 * @param x1 Конец по x
 * @param color Цвет
 */
void lcd_h_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t color);

/**
 * @brief Вертикальная линия
 * @param x0 Начало по x
 * @param y0 Начало по y
 * @param y1 Конец по y
 * @param color Цвет
 */
void lcd_v_line(uint16_t x0, uint16_t y0, uint16_t y1, uint16_t color);

#endif // ILI9488_I80_H