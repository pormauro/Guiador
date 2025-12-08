#pragma once
#include <Arduino.h>

// =========================
// ENCODER
// =========================
#define PIN_ENC_A        16
#define PIN_ENC_B        17
// (Elegidos entre: 5,17,16,4,0,2,15 → solo 16 y 17 sirven sin problemas)

// =========================
// BTS7960 DRIVER
// =========================
#define PIN_RPWM         32
#define PIN_LPWM         33
#define PIN_REN          25
#define PIN_LEN          26

// =========================
// SENSOR DE CORRIENTE (ADC1)
// =========================a
#define PIN_CURRENT_ADC  35   // Solo entrada → perfecto para sensor

// =========================
// ENTRADAS OPTICAS + BOTON
// =========================
#define PIN_OPT_LEFT     18   // L
#define PIN_OPT_RIGHT    19   // R
#define PIN_LIMIT_MAG    21   // Final de carrera
#define PIN_BUTTON       22   // Botón

// =========================
// SALIDA VALVULA/RELE
// =========================
#define PIN_VALVE_OUT    23   // 34 NO sirve → SOLO ENTRADA

// =========================
// LED STATUS
// =========================
#define PIN_LED_STATUS    2   // Cuidado: mantiene requerimientos de boot
