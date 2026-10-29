#ifndef GENERAL_H
#define GENERAL_H

#include <Arduino.h>

enum class MatrixMode : uint8_t {
    WATER = 0,
    HEART = 1,
    LEVEL = 2
};

void general_init(void);
void general_run(void);
MatrixMode general_getMatrixMode(void);
const char *general_getMatrixModeName(void);

#endif
