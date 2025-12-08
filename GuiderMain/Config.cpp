// File: Config.cpp
#include <Arduino.h>
#include "Config.h"

Config gConfig;

static void eepromInit() {
  static bool started = false;
  if (!started) {
    EEPROM.begin(EEPROM_SIZE);
    started = true;
  }
}

void setDefaultConfig() {
  gConfig.magic = CONFIG_MAGIC;

  // Guiador
  gConfig.home_position_deg          = 0.0f;
  gConfig.edge_max_deg               = 12.0f;
  gConfig.k_edge_deg_per_step        = 0.4f;
  gConfig.edge_control_period_ms     = 50;
  gConfig.edge_debounce_ms           = 100;
  gConfig.no_paper_timeout_ms        = 300;
  gConfig.edge_saturation_timeout_ms = 2000;

  // Manual
  gConfig.manual_speed_scale         = 0.5f;

  // PID
  gConfig.pid_kp = 2.0f;
  gConfig.pid_ki = 0.5f;
  gConfig.pid_kd = 0.1f;

  // Corriente
  gConfig.soft_current_limitA = 6.0f;
  gConfig.hard_current_limitA = 8.0f;
  gConfig.current_adc_offset  = 2048;
  gConfig.current_adc_scale   = 0.005f;

  // Encoder
  gConfig.counts_per_degree = 50.0f;
}

void loadConfig() {
  eepromInit();
  EEPROM.get(0, gConfig);

  if (gConfig.magic != CONFIG_MAGIC) {
    setDefaultConfig();
    saveConfig();
    return;
  }

  // Sanitizar nuevos campos añadidos en versiones anteriores
  if (gConfig.manual_speed_scale <= 0.0f || gConfig.manual_speed_scale > 1.0f) {
    gConfig.manual_speed_scale = 0.5f;
  }
}

void saveConfig() {
  eepromInit();
  gConfig.magic = CONFIG_MAGIC;
  EEPROM.put(0, gConfig);
  EEPROM.commit();
}
