#ifndef MAX7219_H
#define MAX7219_H

#include <Arduino.h>
#include "fonts.h"
#include "icons.h"

#define MAX7219_NUM_MODULES 2

#define REG_NOOP         0x00
#define REG_DIGIT0       0x01
#define REG_DECODEMODE   0x09
#define REG_INTENSITY    0x0A
#define REG_SCANLIMIT    0x0B
#define REG_SHUTDOWN     0x0C
#define REG_DISPLAYTEST  0x0F

void MAX7219_Init(void);
void MAX7219_SetBrightness(uint8_t brightness);
uint8_t MAX7219_GetBrightness(void);
void MAX7219_Clear(void);
void MAX7219_Update(void);
void MAX7219_SetPixel(int16_t x, int16_t y, uint8_t state);
void MAX7219_DrawChar(char ch, int16_t x, int16_t y, const FontDef_t *font);
void MAX7219_DrawString(const char *str, int16_t x, int16_t y, const FontDef_t *font);
void MAX7219_DrawIcon(const IconDef_t *icon, int16_t x, int16_t y);

// Blocking convenience helpers. Do not call these from the non-blocking main loop.
void MAX7219_ScrollText(const char *str, uint32_t speed_ms, const FontDef_t *font);
void MAX7219_ScrollIcon(const IconDef_t *icon, uint32_t speed_ms);
void MAX7219_ScrollTextWithIcon(const char *str, const IconDef_t *icon,
                                uint32_t speed_ms, const FontDef_t *font);

#endif
