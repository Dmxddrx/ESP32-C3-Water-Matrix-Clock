#include "icons.h"

static const uint8_t data_Heart[8] = {
    0x00,
    0x66,
    0xFF,
    0xFF,
    0x7E,
    0x3C,
    0x18,
    0x00
};

static const uint8_t data_Smiley[8] = {
    0x00,
    0x00,
    0x24,
    0x00,
    0x00,
    0x42,
    0x3C,
    0x00
};

const IconDef_t Icon_Heart  = {8, 8, data_Heart};
const IconDef_t Icon_Smiley = {8, 8, data_Smiley};
