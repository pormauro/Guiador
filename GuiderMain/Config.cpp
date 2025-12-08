// File: Config.cpp
#include <Arduino.h>
#include "Config.h"
#include <math.h>

Config gConfig;

static void eepromInit() {
  static bool started = false;
  if (!started) {
    EEPROM.begin(EEPROM_SIZE);
    started = true;
  }
}

static bool isFiniteFloat(float v) {
  return !isnan(v) && !isinf(v);
}

static bool isConfigValid(const Config &c) {
  if (c.magic != CONFIG_MAGIC) return false;

  // Chequeos básicos de rango
  if (!isFiniteFloat(c.home_position_deg))          return false;
  if (!isFiniteFloat(c.edge_max_deg)   || c.edge_max_deg <= 0.1f || c.edge_max_deg > 90.0f) return false;
  if (!isFiniteFloat(c.k_edge_deg_per_step) || c.k_edge_deg_per_step <= 0.0f || c.k_edge_deg_per_step > 10.0f) return false;

  if (c.edge_control_period_ms < 10 || c.edge_control_period_ms > 1000) return false;
  if (c.edge_debounce_ms < 10    || c.edge_debounce_ms > 2000) return false;
  if (c.no_paper_timeout_ms < 50 || c.no_paper_timeout_ms > 10000) return false;
  if (c.edge_saturation_timeout_ms < 100 || c.edge_saturation_timeout_ms > 20000) return false;

  if (!isFiniteFloat(c.pid_kp) || !isFiniteFloat(c.pid_ki) || !isFiniteFloat(c.pid_kd)) return false;

  if (!isFiniteFloat(c.soft_current_limitA) || c.soft_current_limitA <= 0.0f || c.soft_current_limitA > 50.0f) return false;
  if (!isFiniteFloat(c.hard_current_limitA) || c.hard_current_limitA <= 0.0f || c.hard_current_limitA > 80.0f) return false;
  if (c.hard_current_limitA <= c.soft_current_limitA) return false;

  if (c.current_adc_offset > 4095) return false;
  if (!isFiniteFloat(c.current_adc_scale) || c.current_adc_scale <= 0.0f || c.current_adc_scale > 0.1f) return false;

  if (!isFiniteFloat(c.counts_per_degree) || c.counts_per_degree < 1.0f || c.counts_per_degree > 100000.0f) return false;
  if (!isFiniteFloat(c.piston_max_travel_deg) || c.piston_max_travel_deg <= 0.0f || c.piston_max_travel_deg > 360.0f) return false;

  return true;
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

  // PID
  gConfig.pid_kp = 2.0f;
  gConfig.pid_ki = 0.5f;
  gConfig.pid_kd = 0.1f;

  // Corriente
  gConfig.soft_current_limitA = 6.0f;
  gConfig.hard_current_limitA = 8.0f;
  gConfig.current_adc_offset  = 2048;
  gConfig.current_adc_scale   = 0.005f;  // 5 mA por cuenta aprox.

  // Encoder
  gConfig.counts_per_degree    = 50.0f;
  gConfig.piston_max_travel_deg = 40.0f;
}

void loadConfig() {
  eepromInit();
  EEPROM.get(0, gConfig);

  if (!isConfigValid(gConfig)) {
    // Cualquier cosa rara en EEPROM → defaults y guardado
    setDefaultConfig();
    saveConfig();
  }
}

void saveConfig() {
  eepromInit();
  gConfig.magic = CONFIG_MAGIC;
  EEPROM.put(0, gConfig);
  EEPROM.commit();
}
