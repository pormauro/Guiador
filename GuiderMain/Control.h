// File: Control.h
#pragma once
#include <Arduino.h>
#include "Pins.h"
#include "Config.h"

struct ServoState {
  volatile long encoderCount;  // cuenta cruda encoder
  float positionDeg;           // posición actual (deg)
  float targetDeg;             // setpoint (deg)

  float pidIntegral;
  float pidLastError;

  float currentA;

  bool faultOCSoft;
  bool faultOCHard;
  bool faultMagLimit;
  bool faultNoPaper;
  bool faultEdgeSat;

  bool paperPresent;
};

struct GuideState {
  float    edgeOffsetDeg;      // offset desde home
  float    servoTargetDeg;     // home + offset

  uint8_t  lastL;
  uint8_t  lastR;
  uint32_t sameStateTimeMs;

  uint32_t noPaperTimeMs;
  uint32_t satTimeMs;
};

extern ServoState gServoState;
extern GuideState gGuideState;

// Inicialización y tareas
void initIO();
void performStartupHoming();
void initControlTasks();

// Estado del servo / fallos
float  getServoPositionDeg();
float  getServoTargetDeg();
float  getCurrentA();
bool   getAnyFault();
String getFaultString();

// Versiones "seguras" para JSON (sanitizadas)
float  getServoPositionDegSafe();
float  getServoTargetDegSafe();
float  getCurrentASafe();

// ---- MODO MANTENIMIENTO / MANUAL ----
void setManualMode(bool enabled);
bool getManualMode();
void setManualCommand(float cmd);     // -1.0 .. 1.0 (izq/der)
void setValveOutput(bool on);
bool getValveOutput();

// Lectura de entradas digitales (para web de mantenimiento)
void getInputsStatus(bool &optL, bool &optR, bool &limitMag, bool &button);
