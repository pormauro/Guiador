// File: Control.h
#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Pins.h"

struct ServoState {
  volatile long encoderCount;  // cuenta cruda del encoder
  float positionDeg;           // posición actual (deg)
  float targetDeg;             // setpoint (deg)

  float pidIntegral;
  float pidLastError;

  float currentA;

  bool faultOCSoft;            // soft overcurrent
  bool faultOCHard;            // hard overcurrent
  bool faultMagLimit;          // límite mecánico
  bool faultNoPaper;           // sin papel
  bool faultEdgeSat;           // saturación guiador

  bool paperPresent;
};

struct GuideState {
  float    edgeOffsetDeg;      // offset respecto de home
  float    servoTargetDeg;     // home + offset

  uint8_t  lastL;
  uint8_t  lastR;
  uint32_t sameStateTimeMs;

  uint32_t noPaperTimeMs;
  uint32_t satTimeMs;
};

extern ServoState gServoState;
extern GuideState gGuideState;

void initIO();
void initControlTasks();

float  getServoPositionDeg();
float  getServoTargetDeg();
float  getCurrentA();
bool   getAnyFault();
String getFaultString();
