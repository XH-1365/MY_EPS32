#pragma once
#include <Arduino.h>

void motionInit();
void motionUpdate();     // 在 loop 里一直调用
void stopAllBrake();

void startMoveX(float mm);
void startMoveY(float mm);
void waitMoveDone();

void moveToAbs(float x_mm, float y_mm);

float curXmm();
float curYmm();
long  curEncX();
long  curEncY();

bool isMovingX();
bool isMovingY();

void zeroEncoders();     // home: encX=encY=0