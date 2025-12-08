// File: Pins.h
#pragma once
#include <Arduino.h>

// === DRIVER BTS7960 43A ===
#define PIN_RPWM 18    // PWM derecha
#define PIN_LPWM 19    // PWM izquierda
#define PIN_REN  21    // enable derecha
#define PIN_LEN  22    // enable izquierda

// === ENCODER INCREMENTAL ===
#define PIN_ENC_A 25
#define PIN_ENC_B 26

// === SENSOR DE CORRIENTE ANALÓGICO ===
#define PIN_CURRENT_ADC 34   // ADC1

// === SENSORES DE PAPEL ===
#define PIN_OPT_LEFT  14
#define PIN_OPT_RIGHT 12

// === FINAL DE CARRERA MAGNÉTICO ===
#define PIN_LIMIT_MAG 27

// === VÁLVULA ===
#define PIN_VALVE_OUT 23

// === BOTÓN LOCAL ===
#define PIN_BUTTON 13

// === LED ONBOARD ===
#define PIN_LED_STATUS 2
