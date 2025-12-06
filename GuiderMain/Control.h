// File: Control.h
#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Pins.h"

struct ServoState {
  volatile long encoderCount;
  float positionDeg;
  float targetDeg;

  float pidIntegral;
  float pidLastError;

  float currentA;

  bool faultOvercurrentSoft;
  bool faultOvercurrentHard;
  bool faultLimitMag;
  bool faultNoPaper;
  bool faultEdgeSaturation;

  bool paperPresent;
};

struct GuideState {
  float edgeOffsetDeg;
  float servoTargetDeg;

  uint32_t sameStateTimeMs;
  uint8_t lastL;
  uint8_t lastR;

  uint32_t noPaperTimeMs;
  uint32_t saturationTimeMs;
};

extern ServoState gServoState;
extern GuideState gGuideState;

void initIO();
void initControlTasks();

float getServoPositionDeg();
float getServoTargetDeg();
float getCurrentA();
bool  getAnyFault();
String getFaultString();
