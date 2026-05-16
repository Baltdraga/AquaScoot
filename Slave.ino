// ===== SLAVE =====/espressif/arduino-esp32/2.0.17/NimBLE/1.4.1
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "driver/ledc.h"

const int PWM_PIN = 5;
const ledc_channel_t PWM_CHANNEL = LEDC_CHANNEL_0;
const ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
const ledc_timer_bit_t PWM_RES = LEDC_TIMER_12_BIT;
const ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;

const int PWM_FREQ = 100;
const float PERIOD_MS = 1000.0f / PWM_FREQ;
const float MIN_PWM = 1.0f;
const float STEP_PWM = 0.2f;
const unsigned long TIMEOUT = 500;
const unsigned long DECAY_INTERVAL = 1000;
const unsigned long STATUS_INTERVAL = 500;
static const char MASTER_PASSWORD[] = "AQS01M"; // expected from MASTER
static const char SLAVE_PASSWORD[]  = "AQS01S"; // sent by SLAVE
static const uint8_t LINK_GROUP_ID = 1; // set unique value per pair (1..255)

enum : uint8_t { MSG_CMD = 0xC1, MSG_STATUS = 0x5A };

// ИСПРАВЛЕНО: Восстановлены массивы [8] и [6]
struct CommandPacket { uint8_t type; uint8_t groupId; char password[8]; uint8_t dstMac[6]; float pulseWidthMs; };
struct StatusPacket  { uint8_t type; uint8_t groupId; char password[8]; float currentPwmMs; };

float currentPWM = MIN_PWM;
bool packetReceived = false;
unsigned long lastPacketTime = 0, lastDecayTime = 0, lastStatusSend = 0;
uint8_t masterMac[6] = {0}; // ИСПРАВЛЕНО: Восстановлен массив [6]
bool masterKnown = false;
bool stopRampActive = false;
unsigned long stopRampStartMs = 0;
unsigned long stopRampDurationMs = 0;
float stopRampFromPwm = MIN_PWM;
bool startRampActive = false;
uint8_t selfMac[6] = {0}; // ИСПРАВЛЕНО: Восстановлен массив [6]

unsigned long startRampStartMs = 0;
unsigned long startRampDurationMs = 0;
float startRampToPwm = MIN_PWM;

static inline float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static inline unsigned long calcStopRampMs(float fromPwm) {
  float ratio = clamp01((fromPwm - 1.2f) / (2.0f - 1.2f));
  return (unsigned long)(1500.0f * ratio);
}

static inline unsigned long calcStartRampMs(float toPwm) {
  float ratio = clamp01((toPwm - 1.2f) / (2.0f - 1.2f));
  return (unsigned long)(1000.0f * ratio);
}

static inline void applyPWM(float width_ms) {
  const int maxDuty = (1 << PWM_RES) - 1;
  const float dutyCycle = width_ms / PERIOD_MS;
  const int duty = (int)(dutyCycle * maxDuty + 0.5f);
  ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL);
  currentPWM = width_ms;
}

void sendStatus();

void setupPWM() {
  ledc_timer_config_t tcfg = {.speed_mode=PWM_MODE,.duty_resolution=PWM_RES,.timer_num=PWM_TIMER,.freq_hz=PWM_FREQ,.clk_cfg=LEDC_AUTO_CLK};
  ledc_timer_config(&tcfg);
  ledc_channel_config_t ccfg = {.gpio_num=PWM_PIN,.speed_mode=PWM_MODE,.channel=PWM_CHANNEL,.intr_type=LEDC_INTR_DISABLE,.timer_sel=PWM_TIMER,.duty=0,.hpoint=0};
  ledc_channel_config(&ccfg);
}

// Адаптировано под ESP32 Core v2.0.17
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len != (int)sizeof(CommandPacket)) return;
  CommandPacket pkt{};
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.type != MSG_CMD) return;
  if (pkt.groupId != LINK_GROUP_ID) return;
  if (strncmp(pkt.password, MASTER_PASSWORD, 6) != 0) return;

  bool discovery = true;
  for (int i = 0; i < 6; ++i) { if (pkt.dstMac[i] != 0) { discovery = false; break; } }

  // Ignore commands targeted to other SLAVE MACs.
  if (!discovery && memcmp(pkt.dstMac, selfMac, 6) != 0) return;

  // In discovery mode, respond once but do not bind/apply control.
  if (discovery) {
    if (!masterKnown) {
      memcpy(masterMac, mac_addr, 6);
      masterKnown = true;
      sendStatus();
      masterKnown = false;
      memset(masterMac, 0, sizeof(masterMac));
    }
    return;
  }

  // Sticky bind: once MASTER MAC is learned, only that MASTER is accepted until reboot.
  if (masterKnown && memcmp(masterMac, mac_addr, 6) != 0) return;
  memcpy(masterMac, mac_addr, 6);
  masterKnown = true;

  if (pkt.pulseWidthMs <= (MIN_PWM + 0.01f) && currentPWM > MIN_PWM) {
    // Drive -> Stop behavior:
    // level2 (<=1.2ms) instant; level6 (2.0ms) -> 1.5s; intermediate proportional.
    if (currentPWM <= 1.2f) {
      applyPWM(MIN_PWM);
      stopRampActive = false;
    } else {
      stopRampFromPwm = currentPWM;
      stopRampDurationMs = calcStopRampMs(stopRampFromPwm);
      stopRampStartMs = millis();
      stopRampActive = (stopRampDurationMs > 0);
      if (!stopRampActive) applyPWM(MIN_PWM);
    }
  } else {
    stopRampActive = false;
    // Stop -> Drive behavior:
    // level2 (<=1.2ms) instant; level6 (2.0ms) from 1.0->2.0 over 1.0s; intermediate proportional.
    if (currentPWM <= (MIN_PWM + 0.01f) && pkt.pulseWidthMs > (MIN_PWM + 0.01f)) {
      if (pkt.pulseWidthMs <= 1.2f) {
        applyPWM(pkt.pulseWidthMs);
        startRampActive = false;
      } else {
        startRampDurationMs = calcStartRampMs(pkt.pulseWidthMs);
        startRampStartMs = millis();
        startRampToPwm = pkt.pulseWidthMs;
        startRampActive = (startRampDurationMs > 0);
        if (!startRampActive) applyPWM(startRampToPwm);
      }
    } else {
      startRampActive = false;
      applyPWM(pkt.pulseWidthMs);
    }
  }

  lastPacketTime = millis();
  packetReceived = true;
}

void sendStatus() {
  if (!masterKnown) return;
  if (!esp_now_is_peer_exist(masterMac)) {
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, masterMac, 6);
    peer.channel = 6;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) return;
  }
  StatusPacket st{};
  st.type = MSG_STATUS;
  st.groupId = LINK_GROUP_ID;
  strncpy(st.password, SLAVE_PASSWORD, sizeof(st.password)-1);
  st.currentPwmMs = currentPWM;
  esp_now_send(masterMac, (uint8_t*)&st, sizeof(st));
}

void setup() {
  WiFi.mode(WIFI_STA);
  esp_wifi_get_mac(WIFI_IF_STA, selfMac);
  setupPWM();
  applyPWM(MIN_PWM);
  lastPacketTime = millis();
  lastDecayTime = lastPacketTime;

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) while (true) { delay(1000); }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  unsigned long now = millis();

  if (stopRampActive) {
    unsigned long elapsed = now - stopRampStartMs;
    if (elapsed >= stopRampDurationMs) {
      applyPWM(MIN_PWM);
      stopRampActive = false;
    } else {
      float k = (float)elapsed / (float)stopRampDurationMs;
      float pwm = stopRampFromPwm + (MIN_PWM - stopRampFromPwm) * k;
      applyPWM(pwm);
    }
  }

  if (startRampActive) {
    unsigned long elapsed = now - startRampStartMs;
    if (elapsed >= startRampDurationMs) {
      applyPWM(startRampToPwm);
      startRampActive = false;
    } else {
      float k = (float)elapsed / (float)startRampDurationMs;
      float pwm = MIN_PWM + (startRampToPwm - MIN_PWM) * k;
      applyPWM(pwm);
    }
  }

  if (packetReceived) {
    if (now - lastPacketTime > TIMEOUT) {
      if (now - lastDecayTime >= DECAY_INTERVAL) {
        lastDecayTime = now;
        if (currentPWM > MIN_PWM) {
          float next = currentPWM - STEP_PWM;
          if (next < MIN_PWM) next = MIN_PWM;
          applyPWM(next);
        }
      }
    } else {
      lastDecayTime = now;
    }
  }

  if (now - lastStatusSend >= STATUS_INTERVAL) {
    lastStatusSend = now;
    sendStatus();
  }
}
