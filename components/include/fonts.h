#ifndef __FONTS_H__
#define __FONTS_H__

#include "stdint.h"

typedef struct
{
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} font_t;

extern font_t font_7x10;
extern font_t font_11x18;
extern font_t font_16x26;
extern font_t font_16x25_rus;

#endif // __FONTS_H__
