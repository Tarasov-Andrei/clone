#ifndef ST7735_H
#define ST7735_H

#include "stdint.h"
#include "fonts.h"

// Color definitions
#define TFT_BLACK 0x0000
#define TFT_BLUE 0x001F
#define TFT_RED 0xF800
#define TFT_GREEN 0x07E0
#define TFT_CYAN 0x07FF
#define TFT_MAGENTA 0xF81F
#define TFT_YELLOW 0xFFE0
#define TFT_WHITE 0xFFFF
#define TFT_LIGHTGREY 0xC618

/**
 * @brief Инициализация устройства
 */
void st7735_init(void);

/**
 * @brief Заливка экрана
 * @param color Цвет
 */
void st7735_fill_screen(uint16_t color);

/**
 * @brief Залитый прямоугольник
 * @param x0 Начало по x
 * @param y0 Начало по y
 * @param x1 Конец по x
 * @param y1 Конец по y
 * @param color Цвет
 */
void st7735_fill_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);

/**
 * @brief Прямоугольник
 * @param x0 Начало по x
 * @param y0 Начало по y
 * @param x1 Конец по x
 * @param y1 Конец по y
 * @param color Цвет
 */
void st7735_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);

/**
 * @brief Горизонтальная линия
 * @param x0 Начало по x
 * @param y0 Начало по y
 * @param x1 Конец по x
 * @param color Цвет
 */
void st7735_h_line(uint8_t x0, uint8_t y0, uint8_t x1, uint16_t color);

/**
 * @brief Вертикальная линия
 * @param x0 Начало по x
 * @param y0 Начало по y
 * @param y1 Конец по y
 * @param color Цвет
 */
void st7735_v_line(uint8_t x0, uint8_t y0, uint8_t y1, uint16_t color);

/**
 * @brief Символ
 * @param x Начало по x
 * @param y Начало по y
 * @param c Символ
 * @param color Цвет символа
 * @param bg Цвет фона
 * @param fonts Шрифт
 * @param size Размер
 */
void st7735_char(uint8_t x, uint8_t y, unsigned char c, uint16_t color, uint16_t bg, font_t fonts, uint8_t size);

/**
 * @brief Русский символ
 * @param x Начало по x
 * @param y Начало по y
 * @param c Символ
 * @param color Цвет символа
 * @param bg Цвет фона
 * @param fonts Шрифт
 * @param size Размер
 */
void st7735_char_rus(uint8_t x, uint8_t y, unsigned char c, uint16_t color, uint16_t bg, font_t fonts, uint8_t size);

/**
 * @brief Печать текста
 * @param x Начало по x
 * @param y Начало по y
 * @param *str Строка
 * @param color Цвет текста
 * @param bg Цвет фона
 * @param font Шрифт
 * @param size Размер
 */
void st7735_print(uint8_t x, uint8_t y, const char *str, uint16_t color, uint16_t bg, font_t font, uint8_t size);

/**
 * @brief Печать русского текста
 * @param x Начало по x
 * @param y Начало по y
 * @param *str Строка
 * @param color Цвет текста
 * @param bg Цвет фона
 * @param font Шрифт
 * @param size Размер
 */
void st7735_print_rus(uint8_t x, uint8_t y, const char *str, uint16_t color, uint16_t bg, font_t font, uint8_t size);

/**
 * @brief Отобразить картинку
 * @param x Начало по x
 * @param y Начало по y
 * @param w Ширина
 * @param h Высота
 * @param *data Указатель на картинку
 */
void st7735_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint32_t *data);

#endif // ST7735_H