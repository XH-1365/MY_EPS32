#pragma once
#include <Arduino.h>

void motorXYBegin();
void motorXYUpdate();

void motorXYHome();        // enc清零
void motorXYStopBrake();   // 急停刹车

void motorMoveX(float mm);
void motorMoveY(float mm);

void motorPrintPos();
