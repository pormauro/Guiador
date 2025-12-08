// File: Control.cpp
#include <Arduino.h>
#include "Control.h"
#include "Pins.h"
#include "Config.h"
#include "Log.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ======================
// ESTADOS GLOBALES
// ======================

ServoState gServoState;
GuideState gGuideState;

// ======================
// CONFIG PWM (LEDC estilo Luis Llamas)
// ======================
//
// const int ledChannel = 0;
// const int freq       = 5000;
// const int resolution = 8;
// ledcSetup(ledChannel, freq, resolution);
// ledcAttachPin(pin, ledChannel);
// ledcWrite(ledChannel, duty);
//
static const uint8_t PWM_CH_RPWM = 0;
static const uint8_t PWM_CH_LPWM = 1;

static const uint32_t PWM_FREQ     = 20000; // 20 kHz
static const uint8_t  PWM_RES_BITS = 10;    // 10 bits
static const uint16_t PWM_MAX      = (1 << PWM_RES_BITS) - 1; // 1023

// ======================
// ESTADO DE MODO MANUAL / AUTO
// ======================

// Modo manual: pisa TODO (PID, auto, botón)
static volatile bool  sManualMode = false;
static volatile float sManualCmd  = 0.0f;   // -1 .. 1 (mandado desde web)

// Ciclo automático (solo válido cuando NO está en manual)
static volatile bool  sAutoRun    = false;  // ON/OFF del ciclo automático

// Estado del relé / válvula
static bool sValveState = false;

// ======================
// PROTOTIPOS INTERNOS
// ======================

static void IRAM_ATTR isrEncA();
static void IRAM_ATTR isrEncB();
static void controlTask(void *pv);
static void guideTask(void *pv);
static void updateCurrentMeasurement();
static void setMotorOutput(float u);

// ======================
// INIT IO
// ======================

void initIO() {
  Serial.println("INIT IO (LEDC + modos MANUAL/AUTO)…");
  logEvent("INIT IO");

  // --- DRIVER BTS7960 ---
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_REN,  OUTPUT);
  pinMode(PIN_LEN,  OUTPUT);

  digitalWrite(PIN_REN, LOW);
  digitalWrite(PIN_LEN, LOW);

  // ==== PWM LEDC ESTILO LUIS LLAMAS ====
  // Configuramos los canales PWM y los asociamos a los pines

  // Canal para RPWM
  ledcSetup(PWM_CH_RPWM, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_RPWM, PWM_CH_RPWM);

  // Canal para LPWM
  ledcSetup(PWM_CH_LPWM, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_LPWM, PWM_CH_LPWM);

  // Duty inicial 0 en ambos
  ledcWrite(PWM_CH_RPWM, 0);
  ledcWrite(PWM_CH_LPWM, 0);

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

  pinMode(PIN_BUTTON, INPUT_PULLUP);  // botón físico (LOW = pulsado)

  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // --- ESTADOS ---
  memset(&gServoState, 0, sizeof(gServoState));
  memset(&gGuideState, 0, sizeof(gGuideState));

  gGuideState.lastL = 2;
  gGuideState.lastR = 2;

  sManualMode = false;
  sManualCmd  = 0.0f;
  sAutoRun    = false;

  logEvent("IO OK (PWM LEDC listo)");
  Serial.println("IO OK (PWM con LEDC, modos listos).");
}

// ======================
// INIT TASKS (CORE 1)
// ======================

void initControlTasks() {
  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    6000,
    nullptr,
    3,
    nullptr,
    1
  );

  xTaskCreatePinnedToCore(
    guideTask,
    "GuideTask",
    5000,
    nullptr,
    2,
    nullptr,
    1
  );

  logEvent("ControlTask y GuideTask iniciadas");
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

  static float filt = 2048;
  filt = 0.9f * filt + 0.1f * raw;

  float I = (filt - gConfig.current_adc_offset) * gConfig.current_adc_scale;

  gServoState.currentA = I;

  bool prevSoft = gServoState.faultOCSoft;
  bool prevHard = gServoState.faultOCHard;

  gServoState.faultOCSoft = (I > gConfig.soft_current_limitA);

  if (I > gConfig.hard_current_limitA) {
    gServoState.faultOCHard = true;
  } else {
    // si baja, liberamos hard fault (si querés latch, sacá esto)
    gServoState.faultOCHard = false;
  }

  if (!prevSoft && gServoState.faultOCSoft) {
    logEvent("SOFT_OC: I=" + String(I, 3));
  }
  if (!prevHard && gServoState.faultOCHard) {
    logEvent("HARD_OC: I=" + String(I, 3));
  }
}

// ======================
// MOTOR BTS7960 (LEDC)
// ======================

static void setMotorOutput(float u) {

  bool fatalFault  = gServoState.faultOCHard;
  bool normalFault = gServoState.faultMagLimit ||
                     gServoState.faultNoPaper  ||
                     gServoState.faultEdgeSat;

  // ============================
  // MODO MANUAL
  // ============================
  // En manual, SOLO cortamos por HARD_OC.
  if (sManualMode) {
    if (fatalFault) {
      digitalWrite(PIN_REN, LOW);
      digitalWrite(PIN_LEN, LOW);
      ledcWrite(PWM_CH_RPWM, 0);
      ledcWrite(PWM_CH_LPWM, 0);
      return;
    }

    // Ignora todos los demás faults en modo manual.
    float scale = constrain(gConfig.manual_speed_scale, 0.0f, 1.0f);
    u = constrain(u, -1.0f, 1.0f) * scale;
    uint16_t duty = (uint16_t)(fabs(u) * PWM_MAX);

    if (fabs(u) < 0.01f) {
      digitalWrite(PIN_REN, LOW);
      digitalWrite(PIN_LEN, LOW);
      ledcWrite(PWM_CH_RPWM, 0);
      ledcWrite(PWM_CH_LPWM, 0);
      return;
    }

    if (u > 0) {
      digitalWrite(PIN_REN, HIGH);
      digitalWrite(PIN_LEN, HIGH);
      ledcWrite(PWM_CH_RPWM, duty);
      ledcWrite(PWM_CH_LPWM, 0);
    } else {
      digitalWrite(PIN_REN, HIGH);
      digitalWrite(PIN_LEN, HIGH);
      ledcWrite(PWM_CH_RPWM, 0);
      ledcWrite(PWM_CH_LPWM, duty);
    }

    return;
  }

  // ============================
  // MODO AUTOMÁTICO (RESPETA FAULTS)
  // ============================
  if (fatalFault || normalFault) {
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, LOW);
    ledcWrite(PWM_CH_RPWM, 0);
    ledcWrite(PWM_CH_LPWM, 0);
    return;
  }

  // Automático normal
  if (fabs(u) < 0.01f) {
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, LOW);
    ledcWrite(PWM_CH_RPWM, 0);
    ledcWrite(PWM_CH_LPWM, 0);
    return;
  }

  u = constrain(u, -1.0f, 1.0f);
  uint16_t duty = (uint16_t)(fabs(u) * PWM_MAX);

  if (u > 0) {
    digitalWrite(PIN_REN, HIGH);
    digitalWrite(PIN_LEN, HIGH);
    ledcWrite(PWM_CH_RPWM, duty);
    ledcWrite(PWM_CH_LPWM, 0);
  } else {
    digitalWrite(PIN_REN, HIGH);
    digitalWrite(PIN_LEN, HIGH);
    ledcWrite(PWM_CH_RPWM, 0);
    ledcWrite(PWM_CH_LPWM, duty);
  }
}

// ======================
// CONTROL TASK (CORE 1)
// ======================

static void controlTask(void *pv) {
  Serial.print("ControlTask running on core ");
  Serial.println(xPortGetCoreID());
  logEvent("ControlTask en core " + String(xPortGetCoreID()));

  // Debounce del botón para el modo AUTOMÁTICO
  bool     lastButtonRaw   = false;
  bool     buttonState     = false;   // estado estable
  uint32_t lastDebounceMs  = 0;

  // Para loggear límites magnéticos una sola vez
  bool prevMagLimit = false;

  while (true) {
    // Posición desde encoder
    long enc = gServoState.encoderCount;
    gServoState.positionDeg = enc / gConfig.counts_per_degree;

    // Corriente
    updateCurrentMeasurement();

    // Límite magnético: refleja el sensor
    bool magNow = (digitalRead(PIN_LIMIT_MAG) == LOW);
    if (magNow != prevMagLimit) {
      if (magNow) logEvent("MAG_LIMIT activado");
      else        logEvent("MAG_LIMIT liberado");
      prevMagLimit = magNow;
    }
    gServoState.faultMagLimit = magNow;

    // Fallos (para LED / info)
    bool anyFault =
      gServoState.faultOCHard  ||
      gServoState.faultMagLimit ||
      gServoState.faultNoPaper  ||
      gServoState.faultEdgeSat;

    // ============================
    // MANEJO DEL BOTÓN (solo AUTO)
    // ============================

    if (!sManualMode) {
      bool raw = (digitalRead(PIN_BUTTON) == LOW);   // LOW = pulsado
      if (raw != lastButtonRaw) {
        lastDebounceMs = millis();
        lastButtonRaw = raw;
      }

      if ((millis() - lastDebounceMs) > 50) {  // debounce 50 ms
        if (raw != buttonState) {
          buttonState = raw;
          // Flanco de bajada: botón presionado → toggle autoRun
          if (buttonState) {
            sAutoRun = !sAutoRun;
            logEvent(String("AutoRun toggled → ") + (sAutoRun ? "ON" : "OFF"));
          }
        }
      }
    } else {
      // En modo manual, el botón NO hace nada y el ciclo automático se apaga
      if (sAutoRun) {
        logEvent("AutoRun forzado OFF por MODO MANUAL");
      }
      sAutoRun = false;
    }

    // ============================
    // CONTROL DEL MOTOR / MODO
    // ============================

    if (sManualMode) {
      // ---- MODO MANUAL: pisa TODO ----
      gServoState.targetDeg = gServoState.positionDeg;  // solo para mostrar
      setMotorOutput(sManualCmd);

    } else {
      // ---- MODO AUTOMÁTICO ----
      gServoState.targetDeg = gGuideState.servoTargetDeg;

      // Manejo de válvula / relé en automático:
      setValveOutput(sAutoRun);

      float error = gServoState.targetDeg - gServoState.positionDeg;

      if (!anyFault && sAutoRun) {
        // PID activo solo si:
        // - no hay fallos
        // - AutoRun está ON
        gServoState.pidIntegral += error * 0.001f;
        gServoState.pidIntegral =
          constrain(gServoState.pidIntegral, -100.0f, 100.0f);

        float d = error - gServoState.pidLastError;
        gServoState.pidLastError = error;

        float u =
          gConfig.pid_kp * error +
          gConfig.pid_ki * gServoState.pidIntegral +
          gConfig.pid_kd * d;

        setMotorOutput(u / 100.0f);
      } else {
        // Auto OFF o fallo → motor parado
        gServoState.pidLastError = 0.0f;
        setMotorOutput(0.0f);
      }
    }

    // ============================
    // LED de estado
    // ============================

    static uint32_t ledMs = 0;
    ledMs += 1;
    if (ledMs >= 200) {
      ledMs = 0;
      if (anyFault)
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
  logEvent("GuideTask en core " + String(xPortGetCoreID()));

  while (true) {
    uint32_t per = gConfig.edge_control_period_ms;
    if (per < 10) per = 10;

    uint8_t L = (digitalRead(PIN_OPT_LEFT)  == LOW);
    uint8_t R = (digitalRead(PIN_OPT_RIGHT) == LOW);

    // Debounce
    if (L == gGuideState.lastL && R == gGuideState.lastR) {
      gGuideState.sameStateTimeMs += per;
    } else {
      gGuideState.sameStateTimeMs = 0;
      gGuideState.lastL = L;
      gGuideState.lastR = R;
    }

    if (gGuideState.sameStateTimeMs >= gConfig.edge_debounce_ms) {
      if (!L && !R) {
        // sin papel
        gGuideState.noPaperTimeMs += per;
        if (gGuideState.noPaperTimeMs >= gConfig.no_paper_timeout_ms) {
          if (!gServoState.faultNoPaper) {
            logEvent("NO_PAPER fault activado");
          }
          gServoState.faultNoPaper = true;
        }
      } else {
        // con papel
        if (gServoState.faultNoPaper) {
          logEvent("NO_PAPER fault limpiado");
        }
        gGuideState.noPaperTimeMs = 0;
        gServoState.faultNoPaper = false;

        if (!gServoState.faultEdgeSat) {
          if (L && !R)
            gGuideState.edgeOffsetDeg += gConfig.k_edge_deg_per_step;
          else if (!L && R)
            gGuideState.edgeOffsetDeg -= gConfig.k_edge_deg_per_step;
          else if (L && R)
            gGuideState.edgeOffsetDeg *= 0.98f;
        }

        gGuideState.edgeOffsetDeg =
          constrain(gGuideState.edgeOffsetDeg,
                    -gConfig.edge_max_deg,
                    gConfig.edge_max_deg);
      }
    }

    gGuideState.servoTargetDeg =
      gConfig.home_position_deg + gGuideState.edgeOffsetDeg;

    vTaskDelay(pdMS_TO_TICKS(per));
  }
}

// ======================
// HELPERS PÚBLICOS
// ======================

float getServoPositionDeg() { return gServoState.positionDeg; }
float getServoTargetDeg()   { return gServoState.targetDeg;   }
float getCurrentA()         { return gServoState.currentA;    }

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

  if (enabled) {
    // Al entrar en manual: apagar auto, motor y relé
    sAutoRun   = false;
    sManualCmd = 0.0f;
    setMotorOutput(0.0f);
    setValveOutput(false);
    logEvent("MANUAL MODE: ON (autoRun OFF, motor OFF, valve OFF)");
  } else {
    // Al salir de manual: dejar todo en estado seguro (auto OFF)
    sManualCmd = 0.0f;
    sAutoRun   = false;
    setMotorOutput(0.0f);
    setValveOutput(false);
    logEvent("MANUAL MODE: OFF (autoRun OFF, motor OFF, valve OFF)");
  }
}

bool getManualMode() {
  return sManualMode;
}

void setManualCommand(float cmd) {
  cmd = constrain(cmd, -1.0f, 1.0f);
  sManualCmd = cmd;
  logEvent("Manual CMD=" + String(cmd, 3));
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
