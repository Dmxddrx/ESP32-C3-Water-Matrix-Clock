#ifndef ICONS_H
#define ICONS_H

#include <stdint.h>

typedef struct {
    uint8_t width;
    uint8_t height;
    const uint8_t *data;  // One byte per row; MSB is the leftmost pixel.
} IconDef_t;

#ifdef __cplusplus
extern "C" {
#endif
extern const IconDef_t Icon_Heart;
extern const IconDef_t Icon_Smiley;
extern const IconDef_t Icon_Power;
#ifdef __cplusplus
}
#endif
#endif
