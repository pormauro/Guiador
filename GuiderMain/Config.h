// File: Config.h
#pragma once
#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_SIZE 512
#define CONFIG_MAGIC 0xDEPRO5AA

struct Config {
  uint32_t magic;

  // GUIADOR / BANDA
  float home_position_deg;
  float edge_max_deg;
  float k_edge_deg_per_step;
  uint32_t edge_control_period_ms;
  uint32_t edge_debounce_ms;
  uint32_t no_paper_timeout_ms;
  uint32_t edge_saturation_timeout_ms;

  // PID
  float pid_kp;
  float pid_ki;
  float pid_kd;

  // CORRIENTE
  float soft_current_limitA;
  float hard_current_limitA;
  uint16_t current_adc_offset;
  float current_adc_scale;

  // ENCODER
  float counts_per_degree;

  uint8_t reserved[32];
};

extern Config gConfig;

void loadConfig();
void saveConfig();
void setDefaultConfig();
