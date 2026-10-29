#include "max7219.h"
#include "project_config.h"
#include <SPI.h>
#include <string.h>

namespace {
uint8_t frameBuffer[MAX7219_NUM_MODULES][8] = {{0}};
uint8_t brightness = ProjectConfig::MATRIX_DEFAULT_BRIGHTNESS;
bool initialized = false;

inline void writePair(uint8_t reg, uint8_t value)
{
    SPI.transfer(reg);
    SPI.transfer(value);
}

void writeBroadcast(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(ProjectConfig::MATRIX_CS_PIN, LOW);
    for (uint8_t i = 0; i < MAX7219_NUM_MODULES; ++i) {
        writePair(reg, value);
    }
    digitalWrite(ProjectConfig::MATRIX_CS_PIN, HIGH);
    SPI.endTransaction();
}
}

void MAX7219_Init(void)
{
    pinMode(ProjectConfig::MATRIX_CS_PIN, OUTPUT);
    digitalWrite(ProjectConfig::MATRIX_CS_PIN, HIGH);
    SPI.begin(ProjectConfig::MATRIX_CLK_PIN, -1, ProjectConfig::MATRIX_MOSI_PIN,
              ProjectConfig::MATRIX_CS_PIN);
    writeBroadcast(REG_SHUTDOWN, 0x00);
    writeBroadcast(REG_DISPLAYTEST, 0x00);
    writeBroadcast(REG_DECODEMODE, 0x00);
    writeBroadcast(REG_SCANLIMIT, 0x07);
    writeBroadcast(REG_INTENSITY, brightness);
    MAX7219_Clear();
    MAX7219_Update();
    writeBroadcast(REG_SHUTDOWN, 0x01);
    initialized = true;
}

void MAX7219_SetBrightness(uint8_t value)
{
    if (value > 15) value = 15;
    brightness = value;
    if (initialized) writeBroadcast(REG_INTENSITY, brightness);
}

uint8_t MAX7219_GetBrightness(void)
{
    return brightness;
}

void MAX7219_Clear(void)
{
    memset(frameBuffer, 0, sizeof(frameBuffer));
}

void MAX7219_SetPixel(int16_t x, int16_t y, uint8_t state)
{
    if (x < 0 || x >= (MAX7219_NUM_MODULES * 8) || y < 0 || y >= 8) return;

    // Preserve the mapping from the supplied STM32 driver: logical left half
    // (x=0..7) is the second/farther module in the daisy chain.
    const uint8_t module = (MAX7219_NUM_MODULES - 1U) - (uint8_t)(x / 8);
    const uint8_t bitPos = 7U - (uint8_t)(x % 8);
    if (state) frameBuffer[module][y] |= (uint8_t)(1U << bitPos);
    else       frameBuffer[module][y] &= (uint8_t)~(1U << bitPos);
}

void MAX7219_Update(void)
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    for (uint8_t row = 0; row < 8; ++row) {
        digitalWrite(ProjectConfig::MATRIX_CS_PIN, LOW);
        // First transmit the farther module's data, then the nearer module's.
        for (int8_t module = MAX7219_NUM_MODULES - 1; module >= 0; --module) {
            writePair((uint8_t)(REG_DIGIT0 + row), frameBuffer[module][row]);
        }
        digitalWrite(ProjectConfig::MATRIX_CS_PIN, HIGH);
    }
    SPI.endTransaction();
}

void MAX7219_DrawChar(char ch, int16_t x, int16_t y, const FontDef_t *font)
{
    if (font == nullptr || font->data == nullptr || ch < 32 || ch > 126) return;
    const uint16_t offset = (uint16_t)(ch - 32) * font->FontHeight;
    for (uint8_t row = 0; row < font->FontHeight; ++row) {
        const uint16_t rowBits = font->data[offset + row];
        for (uint8_t col = 0; col < font->FontWidth && col < 16; ++col) {
            if (rowBits & (uint16_t)(0x8000U >> col)) {
                MAX7219_SetPixel((int16_t)(x + col), (int16_t)(y + row), 1);
            }
        }
    }
}

void MAX7219_DrawString(const char *str, int16_t x, int16_t y, const FontDef_t *font)
{
    if (str == nullptr || font == nullptr) return;
    while (*str) {
        MAX7219_DrawChar(*str++, x, y, font);
        x = (int16_t)(x + font->FontWidth);
    }
}

void MAX7219_DrawIcon(const IconDef_t *icon, int16_t x, int16_t y)
{
    if (icon == nullptr || icon->data == nullptr) return;
    for (uint8_t row = 0; row < icon->height; ++row) {
        const uint8_t rowBits = icon->data[row];
        for (uint8_t col = 0; col < icon->width && col < 8; ++col) {
            if (rowBits & (uint8_t)(0x80U >> col)) {
                MAX7219_SetPixel((int16_t)(x + col), (int16_t)(y + row), 1);
            }
        }
    }
}

void MAX7219_ScrollText(const char *str, uint32_t speed_ms, const FontDef_t *font)
{
    if (str == nullptr || font == nullptr) return;
    const int16_t textWidth = (int16_t)(strlen(str) * font->FontWidth);
    for (int16_t x = (int16_t)(MAX7219_NUM_MODULES * 8); x >= -textWidth; --x) {
        MAX7219_Clear();
        MAX7219_DrawString(str, x, 0, font);
        MAX7219_Update();
        delay(speed_ms);
    }
}

void MAX7219_ScrollIcon(const IconDef_t *icon, uint32_t speed_ms)
{
    if (icon == nullptr) return;
    for (int16_t x = (int16_t)(MAX7219_NUM_MODULES * 8); x >= -(int16_t)icon->width; --x) {
        MAX7219_Clear();
        MAX7219_DrawIcon(icon, x, 0);
        MAX7219_Update();
        delay(speed_ms);
    }
}

void MAX7219_ScrollTextWithIcon(const char *str, const IconDef_t *icon,
                                uint32_t speed_ms, const FontDef_t *font)
{
    if (str == nullptr || icon == nullptr || font == nullptr) return;
    const int16_t textWidth = (int16_t)(strlen(str) * font->FontWidth);
    const int16_t totalWidth = (int16_t)(textWidth + 2 + icon->width);
    for (int16_t x = (int16_t)(MAX7219_NUM_MODULES * 8); x >= -totalWidth; --x) {
        MAX7219_Clear();
        MAX7219_DrawString(str, x, 0, font);
        MAX7219_DrawIcon(icon, (int16_t)(x + textWidth + 2), 0);
        MAX7219_Update();
        delay(speed_ms);
    }
}
