#pragma once
#include <Arduino.h>

void oledInit();
void oledUpdate();   // loop里定时调用即可
void oledSetIp(const String& ipStr);