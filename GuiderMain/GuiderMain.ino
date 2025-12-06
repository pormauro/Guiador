// File: GuiderMain.ino
#include <Arduino.h>
#include "Config.h"
#include "Pins.h"
#include "Control.h"
#include "WebConfig.h"

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\nDEPROS GUIADOR ESP32 - SYSTEM BOOT");

  loadConfig();
  initIO();
  initControlTasks();
  initWiFiAndWeb();

  Serial.println("SYSTEM READY");
}

void loop() {
  webLoop();
  delay(2);
}
