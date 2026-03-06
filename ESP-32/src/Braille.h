#pragma once
#include <Arduino.h>

void brailleQueueText(const String& text);
void brailleStart();
void brailleStop();

void brailleProcess();     // loop里调用
bool brailleIsPrinting();

int  brailleDotI();
int  brailleDotN();