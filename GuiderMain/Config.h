// File: Config.h
#pragma once
#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_SIZE 512
#define CONFIG_MAGIC 0xDEAD55AA   // Hex válido, no causa errores

struct Config {
  uint32_t magic;

  // Posición y guiador
  float    home_position_deg;
  float    edge_max_deg;
  float    k_edge_deg_per_step;
  uint32_t edge_control_period_ms;
  uint32_t edge_debounce_ms;
  uint32_t no_paper_timeout_ms;
  uint32_t edge_saturation_timeout_ms;

  // Manual
  float    manual_speed_scale;   // 0..1 factor para el PWM en manual

  // PID
  float pid_kp;
  float pid_ki;
  float pid_kd;

  // Corriente
  float    soft_current_limitA;
  float    hard_current_limitA;
  uint16_t current_adc_offset;
  float    current_adc_scale;

  // Encoder
  float counts_per_degree;
};

extern Config gConfig;

void setDefaultConfig();
void loadConfig();
void saveConfig();
