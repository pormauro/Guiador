// File: GuiderMain.ino
#include <Arduino.h>
#include "Config.h"
#include "Control.h"
#include "WebConfig.h"

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("DEPROS GUIADOR ESP32 - BOOT");

  // Carga configuración desde EEPROM (o setea default)
  loadConfig();

  // Inicializa IO (pines, encoder, sensores, etc.)
  initIO();

  // Crea tareas FreeRTOS para control y guiador
  initControlTasks();

  // Inicia WiFi en modo AP + servidor HTTP de configuración
  initWiFiAndWeb();

  Serial.println("SYSTEM READY");
}

void loop() {
  // Atiende peticiones HTTP (config / status)
  webLoop();

  // Cede CPU al scheduler / WDT core 1
  delay(2);
}
