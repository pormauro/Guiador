#pragma once
#include <Arduino.h>

// Agrega una línea al log, con timestamp en ms
void logEvent(const String &msg);

// Construye un JSON array con todas las entradas del log:
//   ["[123 ms] ..","[456 ms] ..", ...]
void getLogJson(String &outJsonArray);

// Limpia el buffer de log
void clearLog();
