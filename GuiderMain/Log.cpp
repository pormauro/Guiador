#include "Log.h"

static const int LOG_CAPACITY = 100;

static String sLogBuffer[LOG_CAPACITY];
static int    sLogHead  = 0;  // próxima posición de escritura
static int    sLogCount = 0;  // cantidad de entradas usadas

void logEvent(const String &msg) {
  uint32_t t = millis();
  String line = "[" + String(t) + " ms] " + msg;

  sLogBuffer[sLogHead] = line;
  sLogHead = (sLogHead + 1) % LOG_CAPACITY;
  if (sLogCount < LOG_CAPACITY) {
    sLogCount++;
  }
}

void getLogJson(String &out) {
  out = "[";
  int total = sLogCount;

  for (int i = 0; i < total; ++i) {
    int idx = sLogHead - sLogCount + i;
    if (idx < 0) idx += LOG_CAPACITY;

    if (i > 0) out += ",";

    out += "\"";
    const String &entry = sLogBuffer[idx];
    // Escape básico de comillas y backslash
    for (size_t j = 0; j < entry.length(); ++j) {
      char c = entry[j];
      if (c == '\"' || c == '\\') {
        out += '\\';
      }
      out += c;
    }
    out += "\"";
  }

  out += "]";
}

void clearLog() {
  sLogHead  = 0;
  sLogCount = 0;
}
