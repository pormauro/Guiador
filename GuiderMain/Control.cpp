// File: Control.cpp
#include <Arduino.h>
#include "Control.h"
#include "Pins.h"
#include "Config.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ======================
// ESTADOS GLOBALES
// ======================

ServoState gServoState;
GuideState gGuideState;

// ======================
// CONFIG PWM SIMPLE
// ======================

static const uint8_t PWM_MAX = 255;

// ======================
// ESTADO DE MODO MANUAL
// ======================

static volatile bool  sManualMode = false;
static volatile float sManualCmd  = 0.0f;  // -1 .. 1

static bool sValveState = false;

// ======================
// ESTADO DE OPERACIÓN
// ======================

static volatile bool sOperationEnabled = false;
static float         sStartupHoldDeg   = 0.0f;

// ======================
// HELPERS INTERNOS
// ======================

static float sanitizeFloatInternal(float v,
                                   float fallback = 0.0f,
                                   float minV = -100000.0f,
                                   float maxV =  100000.0f) {
  if (isnan(v) || isinf(v)) return fallback;
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

// ======================
// PROTOTIPOS INTERNOS
// ======================

static void IRAM_ATTR isrEncA();
static void IRAM_ATTR isrEncB();
static void controlTask(void *pv);
static void guideTask(void *pv);
static void updateCurrentMeasurement();
static void setMotorOutput(float u);
static void moveToDegBlocking(float targetDeg, uint32_t timeoutMs);
static void updateStartButtonState();

// ======================
// INIT IO
// ======================

void initIO() {
  Serial.println("INIT IO (sin LEDC)...");

  // --- DRIVER BTS7960 ---
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_REN,  OUTPUT);
  pinMode(PIN_LEN,  OUTPUT);

  digitalWrite(PIN_REN, LOW);
  digitalWrite(PIN_LEN, LOW);

  analogWrite(PIN_RPWM, 0);
  analogWrite(PIN_LPWM, 0);

  // --- ENCODER ---
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncB, CHANGE);

  // --- SENSOR DE CORRIENTE ---
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_CURRENT_ADC, ADC_11db);

  // --- SENSORES GUIADOR ---
  pinMode(PIN_OPT_LEFT,  INPUT_PULLUP);
  pinMode(PIN_OPT_RIGHT, INPUT_PULLUP);
  pinMode(PIN_LIMIT_MAG, INPUT_PULLUP);

  // --- SALIDAS / BOTÓN / LED ---
  pinMode(PIN_VALVE_OUT, OUTPUT);
  digitalWrite(PIN_VALVE_OUT, LOW);
  sValveState = false;

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // --- ESTADOS ---
  memset(&gServoState, 0, sizeof(gServoState));
  memset(&gGuideState, 0, sizeof(gGuideState));

  gGuideState.lastL = 2;
  gGuideState.lastR = 2;

  gServoState.encoderCount  = 0;
  gServoState.positionDeg   = 0.0f;
  gServoState.targetDeg     = 0.0f;
  gServoState.pidIntegral   = 0.0f;
  gServoState.pidLastError  = 0.0f;
  gServoState.currentA      = 0.0f;
  gServoState.faultOCSoft   = false;
  gServoState.faultOCHard   = false;
  gServoState.faultMagLimit = false;
  gServoState.faultNoPaper  = false;
  gServoState.faultEdgeSat  = false;
  gServoState.paperPresent  = false;

  Serial.println("IO OK (PWM con analogWrite).");
}

// ======================
// SECUENCIA DE HOMING
// ======================

void performStartupHoming() {
  Serial.println("Inicio de homing de pistón...");

  // Limpia estados previos
  gServoState.faultMagLimit = false;
  gServoState.faultNoPaper  = false;
  gServoState.faultEdgeSat  = false;
  gServoState.faultOCHard   = false;
  gServoState.faultOCSoft   = false;

  // Búsqueda hacia la izquierda hasta final de carrera
  uint32_t start = millis();
  setMotorOutput(-0.4f);
  while (digitalRead(PIN_LIMIT_MAG) != LOW && (millis() - start) < 6000) {
    delay(5);
  }
  setMotorOutput(0.0f);

  if (digitalRead(PIN_LIMIT_MAG) == LOW) {
    Serial.println("Final de carrera detectado, fijando referencia 0.");
    gServoState.encoderCount = 0;
    gServoState.positionDeg  = 0.0f;
  } else {
    Serial.println("Advertencia: no se detectó el final de carrera (timeout).");
    gServoState.faultMagLimit = true;
  }

  sStartupHoldDeg           = gConfig.piston_max_travel_deg * 0.5f;
  sStartupHoldDeg           = sanitizeFloatInternal(sStartupHoldDeg, 20.0f, 0.0f, gConfig.piston_max_travel_deg);
  gGuideState.edgeOffsetDeg = 0.0f;
  gGuideState.servoTargetDeg = sStartupHoldDeg;

  moveToDegBlocking(sStartupHoldDeg, 4000);

  // Requiere habilitación por botón para iniciar guiador
  sOperationEnabled = false;
  Serial.println("Homing finalizado. Esperando pulsación de botón para operar.");
}

// ======================
// INIT TASKS (CORE 1)
// ======================

void initControlTasks() {
  // ControlTask → CORE 1
  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    6000,
    nullptr,
    2,
    nullptr,
    1
  );

  // GuideTask → CORE 1
  xTaskCreatePinnedToCore(
    guideTask,
    "GuideTask",
    5000,
    nullptr,
    1,
    nullptr,
    1
  );

  Serial.println("Tasks running on Core 1.");
}

// ======================
// ISR ENCODER
// ======================

static void IRAM_ATTR isrEncA() {
  bool A = digitalRead(PIN_ENC_A);
  bool B = digitalRead(PIN_ENC_B);
  gServoState.encoderCount += (A == B) ? 1 : -1;
}

static void IRAM_ATTR isrEncB() {
  bool A = digitalRead(PIN_ENC_A);
  bool B = digitalRead(PIN_ENC_B);
  gServoState.encoderCount += (A != B) ? 1 : -1;
}

// ======================
// CORRIENTE
// ======================

static void updateCurrentMeasurement() {
  uint16_t raw = analogRead(PIN_CURRENT_ADC);

  static float filt = 2048.0f;
  filt = 0.9f * filt + 0.1f * raw;

  float I = (filt - (float)gConfig.current_adc_offset) * gConfig.current_adc_scale;
  I = sanitizeFloatInternal(I, 0.0f, -100.0f, 100.0f);

  gServoState.currentA   = I;
  gServoState.faultOCSoft = (I > gConfig.soft_current_limitA);

  if (I > gConfig.hard_current_limitA) {
    gServoState.faultOCHard = true;
  }
}

// ======================
// MOTOR BTS7960
// ======================

static void setMotorOutput(float u) {
  bool fault =
    gServoState.faultOCHard   ||
    gServoState.faultMagLimit ||
    gServoState.faultNoPaper  ||
    gServoState.faultEdgeSat;

  if (fault) {
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, LOW);
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, 0);
    return;
  }

  if (fabs(u) < 0.01f) {
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, LOW);
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, 0);
    return;
  }

  u = constrain(u, -1.0f, 1.0f);
  uint8_t pwm = (uint8_t)(fabs(u) * PWM_MAX);

  if (u > 0) {
    digitalWrite(PIN_REN, HIGH);
    digitalWrite(PIN_LEN, HIGH);
    analogWrite(PIN_RPWM, pwm);
    analogWrite(PIN_LPWM, 0);
  } else {
    digitalWrite(PIN_REN, HIGH);
    digitalWrite(PIN_LEN, HIGH);
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, pwm);
  }
}

// Mueve el pistón a un ángulo simple (sin PID completo) durante el homing
static void moveToDegBlocking(float targetDeg, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    float cpd = gConfig.counts_per_degree;
    if (cpd < 1.0f || isnan(cpd) || isinf(cpd)) cpd = 50.0f;

    float posDeg = (float)gServoState.encoderCount / cpd;
    float error  = targetDeg - posDeg;
    if (fabs(error) < 0.2f) break;

    float u = constrain(error * 0.04f, -0.6f, 0.6f);
    setMotorOutput(u);
    delay(10);
  }
  setMotorOutput(0.0f);
}

static void updateStartButtonState() {
  static bool last = true; // HIGH = no presionado (por pullup)
  bool now = (digitalRead(PIN_BUTTON) == LOW);

  // Detecta flanco descendente (presionado)
  if (now && !last) {
    // TOGGLE de auto
    sOperationEnabled = !sOperationEnabled;

    // Manejo del relé según modo auto
    if (sOperationEnabled) {
      setValveOutput(true);   // relé ON en AUTO
    } else {
      setValveOutput(false);  // relé OFF al salir de AUTO
    }

    Serial.printf("AUTO MODE = %d\n", sOperationEnabled);
  }

  last = now;
}


// ======================
// CONTROL TASK (CORE 1)
// ======================

static void controlTask(void *pv) {
  Serial.print("ControlTask running on core ");
  Serial.println(xPortGetCoreID());

  while (true) {


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (digitalRead(PIN_BUTTON) == LOW) {
      Serial.println("BOTON DETECTADO");
    }

    updateStartButtonState();

    // Posición desde encoder
    float cpd = gConfig.counts_per_degree;
    if (cpd < 1.0f || isnan(cpd) || isinf(cpd)) cpd = 50.0f;

    long enc = gServoState.encoderCount;
    float pos = (float)enc / cpd;
    gServoState.positionDeg = sanitizeFloatInternal(pos, 0.0f, -1000.0f, 1000.0f);

    // Corriente
    updateCurrentMeasurement();

    // Límite magnético
    if (digitalRead(PIN_LIMIT_MAG) == LOW) {
      gServoState.faultMagLimit = true;
    }

    // Fallos "duros"
    bool blocked =
      gServoState.faultOCHard   ||
      gServoState.faultMagLimit ||
      gServoState.faultNoPaper  ||
      gServoState.faultEdgeSat;

    if (sManualMode) {
      // ---- MODO MANUAL ----
      gServoState.targetDeg = gServoState.positionDeg; // solo para mostrar
      setMotorOutput(sManualCmd);
    } else {
      if (!sOperationEnabled) {
        gGuideState.servoTargetDeg = sStartupHoldDeg;
      }

      // ---- MODO AUTOMÁTICO (PID) ----
      gServoState.targetDeg = gGuideState.servoTargetDeg;
      gServoState.targetDeg = sanitizeFloatInternal(
        gServoState.targetDeg,
        sStartupHoldDeg,
        0.0f,
        gConfig.piston_max_travel_deg
      );

      float error = gServoState.targetDeg - gServoState.positionDeg;

      if (!blocked) {
        gServoState.pidIntegral += error * 0.001f;
        gServoState.pidIntegral = constrain(gServoState.pidIntegral, -100.0f, 100.0f);
      }

      float d = error - gServoState.pidLastError;
      gServoState.pidLastError = error;

      float u =
        gConfig.pid_kp * error +
        gConfig.pid_ki * gServoState.pidIntegral +
        gConfig.pid_kd * d;

      setMotorOutput(u / 100.0f);
    }

    // Indicador LED si hay fallo mayor
    static uint32_t ledMs = 0;
    ledMs += 1;
    if (ledMs >= 200) {
      ledMs = 0;
      if (blocked)
        digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
      else
        digitalWrite(PIN_LED_STATUS, LOW);
    }

    // Evita watchdog
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ======================
// GUIDE TASK (CORE 1)
// ======================

static void guideTask(void *pv) {
  Serial.print("GuideTask running on core ");
  Serial.println(xPortGetCoreID());

  // Debounce FIR de 2 muestras por canal
  uint8_t Lf = 0, Rf = 0;
  bool L = false, R = false;

  while (true) {

    uint32_t per = gConfig.edge_control_period_ms;
    if (per < 10) per = 10;

    // ============================
    // LECTURA CRUDA DE SENSORES
    // ============================
    bool Lraw = (digitalRead(PIN_OPT_LEFT)  == LOW);
    bool Rraw = (digitalRead(PIN_OPT_RIGHT) == LOW);

    // ============================
    // DEBOUNCE INDUSTRIAL FIR
    // ============================
    Lf = ((Lf << 1) | (Lraw ? 1 : 0)) & 0x03;
    Rf = ((Rf << 1) | (Rraw ? 1 : 0)) & 0x03;

    L = (Lf == 0x03);  
    R = (Rf == 0x03);

    // =====================================
    // SIN AUTO Y SIN MANUAL → SOLO SOSTENER
    // =====================================
    if (!sOperationEnabled && !sManualMode) {
      gGuideState.servoTargetDeg = sStartupHoldDeg;
      vTaskDelay(pdMS_TO_TICKS(per));
      continue;
    }

    // ============================
    // DETECCIÓN "NO PAPER"
    // ============================
    if (!L && !R) {
      gGuideState.noPaperTimeMs += per;

      if (gGuideState.noPaperTimeMs >= gConfig.no_paper_timeout_ms)
        gServoState.faultNoPaper = true;

    } else {
      gGuideState.noPaperTimeMs = 0;
      gServoState.faultNoPaper = false;

      // ============================
      // CÁLCULO CORREGIDO DEL OFFSET
      // ============================
      if (!gServoState.faultEdgeSat) {

        if (L && !R) {
          // BANDA A LA IZQUIERDA → mover pistón +offset
          gGuideState.edgeOffsetDeg += gConfig.k_edge_deg_per_step;
        }

        else if (!L && R) {
          // BANDA A LA DERECHA → mover pistón -offset
          gGuideState.edgeOffsetDeg -= gConfig.k_edge_deg_per_step;
        }

        else if (L && R) {
          // Papel centrado → amortiguación suave
          gGuideState.edgeOffsetDeg *= 0.98f;
        }
      }

      // Limitar dentro de rango permitido
      gGuideState.edgeOffsetDeg =
        constrain(gGuideState.edgeOffsetDeg,
                  -gConfig.edge_max_deg, gConfig.edge_max_deg);
    }

    // ============================
    // CÁLCULO FINAL DEL TARGET
    // ============================
    gGuideState.servoTargetDeg =
      gConfig.home_position_deg + gGuideState.edgeOffsetDeg;

    gGuideState.servoTargetDeg = sanitizeFloatInternal(
      gGuideState.servoTargetDeg,
      sStartupHoldDeg,
      0.0f,
      gConfig.piston_max_travel_deg
    );

    vTaskDelay(pdMS_TO_TICKS(per));
  }
}



// ======================
// HELPERS PÚBLICOS
// ======================

float getServoPositionDeg() { return gServoState.positionDeg; }
float getServoTargetDeg()   { return gServoState.targetDeg;   }
float getCurrentA()         { return gServoState.currentA;    }

float getServoPositionDegSafe() {
  return sanitizeFloatInternal(gServoState.positionDeg, 0.0f, -1000.0f, 1000.0f);
}

float getServoTargetDegSafe() {
  return sanitizeFloatInternal(gServoState.targetDeg, 0.0f, -1000.0f, 1000.0f);
}

float getCurrentASafe() {
  return sanitizeFloatInternal(gServoState.currentA, 0.0f, -100.0f, 100.0f);
}

bool getAnyFault() {
  return gServoState.faultOCHard ||
         gServoState.faultMagLimit ||
         gServoState.faultNoPaper ||
         gServoState.faultEdgeSat;
}

String getFaultString() {
  String s;
  if (gServoState.faultOCHard)   s += "HARD_OC;";
  if (gServoState.faultMagLimit) s += "MAG_LIMIT;";
  if (gServoState.faultNoPaper)  s += "NO_PAPER;";
  if (gServoState.faultEdgeSat)  s += "EDGE_SAT;";
  if (!s.length()) s = "OK";
  return s;
}

// ---- MODO MANUAL / MANTENIMIENTO ----

void setManualMode(bool enabled) {
  sManualMode = enabled;
  if (!enabled) {
    sManualCmd = 0.0f;
    setMotorOutput(0.0f);
  } else {
    setValveOutput(false);
    sOperationEnabled = false;
  }

}

bool getManualMode() {
  return sManualMode;
}

void setManualCommand(float cmd) {
  cmd = constrain(cmd, -1.0f, 1.0f);
  sManualCmd = cmd;
}

void setValveOutput(bool on) {
  sValveState = on;
  digitalWrite(PIN_VALVE_OUT, on ? HIGH : LOW);
}

bool getValveOutput() {
  return sValveState;
}

void getInputsStatus(bool &optL, bool &optR, bool &limitMag, bool &button) {
  optL     = (digitalRead(PIN_OPT_LEFT)  == LOW);
  optR     = (digitalRead(PIN_OPT_RIGHT) == LOW);
  limitMag = (digitalRead(PIN_LIMIT_MAG) == LOW);
  button   = (digitalRead(PIN_BUTTON)    == LOW);
}
