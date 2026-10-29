#include <Arduino.h>
#include "general.h"

void setup()
{
    Serial.begin(115200);
    delay(150);

    general_init();
}

void loop()
{
    general_run();
}