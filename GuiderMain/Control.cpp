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
  Serial.println("INIT IO (sin LEDC)...");

  // --- DRIVER BTS7960 ---
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_REN,  OUTPUT);
  pinMode(PIN_LEN,  OUTPUT);

  digitalWrite(PIN_REN, LOW);
  digitalWrite(PIN_LEN, LOW);

  // PWM inicial
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

  // --- SALIDAS ---
  pinMode(PIN_VALVE_OUT, OUTPUT);
  digitalWrite(PIN_VALVE_OUT, LOW);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // --- ESTADOS ---
  memset(&gServoState, 0, sizeof(gServoState));
  memset(&gGuideState, 0, sizeof(gGuideState));

  gGuideState.lastL = 2;
  gGuideState.lastR = 2;

  Serial.println("IO OK (PWM con analogWrite).");
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
    3,
    nullptr,
    1
  );

  // GuideTask → CORE 1
  xTaskCreatePinnedToCore(
    guideTask,
    "GuideTask",
    5000,
    nullptr,
    2,
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

  static float filt = 2048;
  filt = 0.9f * filt + 0.1f * raw;

  float I = (filt - gConfig.current_adc_offset) * gConfig.current_adc_scale;

  gServoState.currentA = I;
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
    gServoState.faultOCHard  ||
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
    digitalWrite(PIN_LEN, LOW);
    analogWrite(PIN_RPWM, pwm);
    analogWrite(PIN_LPWM, 0);
  } else {
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, HIGH);
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, pwm);
  }
}

// ======================
// CONTROL TASK (CORE 1)
// ======================

static void controlTask(void *pv) {
  Serial.print("ControlTask running on core ");
  Serial.println(xPortGetCoreID());

  while (true) {

    long enc = gServoState.encoderCount;
    gServoState.positionDeg = enc / gConfig.counts_per_degree;

    updateCurrentMeasurement();

    if (digitalRead(PIN_LIMIT_MAG) == LOW)
      gServoState.faultMagLimit = true;

    gServoState.targetDeg = gGuideState.servoTargetDeg;

    float error = gServoState.targetDeg - gServoState.positionDeg;

    bool blocked =
      gServoState.faultOCHard ||
      gServoState.faultMagLimit ||
      gServoState.faultNoPaper ||
      gServoState.faultEdgeSat;

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

    // Indicador LED si hay fallo mayor
    static uint32_t ledMs = 0;
    ledMs += 1;
    if (ledMs >= 200) {
      ledMs = 0;

      if (blocked) {
        // Toggle LED correctamente
        digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
      } else {
        digitalWrite(PIN_LED_STATUS, LOW);
      }
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
        if (gGuideState.noPaperTimeMs >= gConfig.no_paper_timeout_ms)
          gServoState.faultNoPaper = true;
      } else {
        // con papel
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
                    -gConfig.edge_max_deg, gConfig.edge_max_deg);
      }
    }

    gGuideState.servoTargetDeg =
      gConfig.home_position_deg + gGuideState.edgeOffsetDeg;

    vTaskDelay(pdMS_TO_TICKS(per));
  }
}

// ======================
// HELPERS
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
