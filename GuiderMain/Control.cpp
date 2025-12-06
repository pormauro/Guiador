// File: Control.cpp
#include "Control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

ServoState gServoState;
GuideState gGuideState;

static const float CONTROL_PERIOD_MS = 1.0f;
static const uint8_t PWM_CH_R = 0;
static const uint8_t PWM_CH_L = 1;
static const uint32_t PWM_FREQ = 20000;
static const uint8_t PWM_BITS = 10;

// Prototipos ISR
static void IRAM_ATTR isrEncA();
static void IRAM_ATTR isrEncB();

// Prototipos tareas
static void controlTask(void *pv);
static void guideTask(void *pv);

void initIO() {
  // BTS7960 pines
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_REN, OUTPUT);
  pinMode(PIN_LEN, OUTPUT);

  digitalWrite(PIN_REN, LOW);
  digitalWrite(PIN_LEN, LOW);

  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_BITS);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_RPWM, PWM_CH_R);
  ledcAttachPin(PIN_LPWM, PWM_CH_L);

  // Encoder
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncB, CHANGE);

  // Corriente
  pinMode(PIN_CURRENT_ADC, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_CURRENT_ADC, ADC_11db);

  // Sensores
  pinMode(PIN_LIMIT_MAG, INPUT_PULLUP);
  pinMode(PIN_OPT_LEFT, INPUT_PULLUP);
  pinMode(PIN_OPT_RIGHT, INPUT_PULLUP);

  pinMode(PIN_VALVE_OUT, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED_STATUS, OUTPUT);

  gServoState = {};
  gGuideState = {};
}

void initControlTasks() {
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(guideTask, "GuideTask", 4096, NULL, 2, NULL, 0);
}

// ISR encoder
static void IRAM_ATTR isrEncA() {
  bool a = digitalRead(PIN_ENC_A);
  bool b = digitalRead(PIN_ENC_B);
  gServoState.encoderCount += (a == b) ? 1 : -1;
}

static void IRAM_ATTR isrEncB() {
  bool a = digitalRead(PIN_ENC_A);
  bool b = digitalRead(PIN_ENC_B);
  gServoState.encoderCount += (a != b) ? 1 : -1;
}

// Control BTS7960
static void setMotorOutput(float u) {
  if (gServoState.faultOvercurrentHard ||
      gServoState.faultLimitMag ||
      gServoState.faultNoPaper ||
      gServoState.faultEdgeSaturation) {
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, LOW);
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);
    return;
  }

  if (abs(u) < 0.01f) {
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, LOW);
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);
    return;
  }

  u = constrain(u, -1.0f, 1.0f);
  uint32_t pwm = abs(u) * ((1 << PWM_BITS) - 1);

  if (u > 0) {
    digitalWrite(PIN_REN, HIGH);
    digitalWrite(PIN_LEN, LOW);
    ledcWrite(PWM_CH_R, pwm);
    ledcWrite(PWM_CH_L, 0);
  } else {
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, HIGH);
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, pwm);
  }
}

static void updateCurrentMeasurement() {
  uint16_t raw = analogRead(PIN_CURRENT_ADC);
  static float filt = raw;
  filt = 0.9f * filt + 0.1f * raw;

  float I = (filt - gConfig.current_adc_offset) * gConfig.current_adc_scale;
  gServoState.currentA = I;

  gServoState.faultOvercurrentSoft = (I > gConfig.soft_current_limitA);
  if (I > gConfig.hard_current_limitA)
    gServoState.faultOvercurrentHard = true;
}

// 1 kHz control de servo
static void controlTask(void *pv) {
  TickType_t last = xTaskGetTickCount();
  TickType_t dt = pdMS_TO_TICKS((uint32_t)CONTROL_PERIOD_MS);

  while (1) {
    long enc = gServoState.encoderCount;
    gServoState.positionDeg = enc / gConfig.counts_per_degree;

    updateCurrentMeasurement();

    if (digitalRead(PIN_LIMIT_MAG) == LOW)
      gServoState.faultLimitMag = true;

    gServoState.targetDeg = gGuideState.servoTargetDeg;

    float error = gServoState.targetDeg - gServoState.positionDeg;
    gServoState.pidIntegral += error * (CONTROL_PERIOD_MS / 1000.0f);
    gServoState.pidIntegral = constrain(gServoState.pidIntegral, -100, +100);

    float d = (error - gServoState.pidLastError) / (CONTROL_PERIOD_MS / 1000.0f);
    gServoState.pidLastError = error;

    float u = 
      gConfig.pid_kp * error +
      gConfig.pid_ki * gServoState.pidIntegral +
      gConfig.pid_kd * d;

    setMotorOutput(u / 100.0f);

    vTaskDelayUntil(&last, dt);
  }
}

// Control de guiador
static void guideTask(void *pv) {
  TickType_t last = xTaskGetTickCount();

  while (1) {
    uint32_t per = gConfig.edge_control_period_ms;
    if (per < 10) per = 10;
    TickType_t dt = pdMS_TO_TICKS(per);

    uint8_t L = digitalRead(PIN_OPT_LEFT)  == LOW;
    uint8_t R = digitalRead(PIN_OPT_RIGHT) == LOW;

    if (L == gGuideState.lastL && R == gGuideState.lastR)
      gGuideState.sameStateTimeMs += per;
    else {
      gGuideState.sameStateTimeMs = 0;
      gGuideState.lastL = L;
      gGuideState.lastR = R;
    }

    if (gGuideState.sameStateTimeMs >= gConfig.edge_debounce_ms) {
      if (!L && !R) {
        gGuideState.noPaperTimeMs += per;
        if (gGuideState.noPaperTimeMs >= gConfig.no_paper_timeout_ms) {
          gServoState.faultNoPaper = true;
        }
      } else {
        gGuideState.noPaperTimeMs = 0;
        gServoState.faultNoPaper = false;

        if (!gServoState.faultEdgeSaturation) {
          if (L && !R)
            gGuideState.edgeOffsetDeg += gConfig.k_edge_deg_per_step;
          else if (!L && R)
            gGuideState.edgeOffsetDeg -= gConfig.k_edge_deg_per_step;
          else if (L && R)
            gGuideState.edgeOffsetDeg *= 0.99f;

          gGuideState.edgeOffsetDeg =
            constrain(gGuideState.edgeOffsetDeg,
                      -gConfig.edge_max_deg,
                      +gConfig.edge_max_deg);
        }
      }
    }

    gGuideState.servoTargetDeg = gConfig.home_position_deg + gGuideState.edgeOffsetDeg;

    vTaskDelayUntil(&last, dt);
  }
}

// Helpers
float getServoPositionDeg() { return gServoState.positionDeg; }
float getServoTargetDeg()   { return gServoState.targetDeg; }
float getCurrentA()         { return gServoState.currentA; }

bool getAnyFault() {
  return gServoState.faultOvercurrentHard ||
         gServoState.faultLimitMag ||
         gServoState.faultNoPaper ||
         gServoState.faultEdgeSaturation;
}

String getFaultString() {
  String s;
  if (gServoState.faultOvercurrentHard)   s += "HARD_OC;";
  if (gServoState.faultLimitMag)          s += "LIMIT_MAG;";
  if (gServoState.faultNoPaper)           s += "NO_PAPER;";
  if (gServoState.faultEdgeSaturation)    s += "EDGE_SAT;";
  if (s == "") s = "OK";
  return s;
}
