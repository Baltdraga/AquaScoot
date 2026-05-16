// ===== MASTER =====/espressif/arduino-esp32/2.0.17/NimBLE/1.4.1
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>
#include <cstring>

#define I2C_SDA 8
#define I2C_SCL 9
#define BTN_START 1
#define BTN_PLUS  2
#define BTN_MINUS 3

static const char MASTER_PASSWORD[] = "AQS01M";
static const char SLAVE_PASSWORD[]  = "AQS01S";
static const uint8_t LINK_GROUP_ID = 1;

uint8_t broadcastMAC[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static const float MIN_PWM = 1.0f;
static const unsigned long SEND_INTERVAL  = 200;
static const unsigned long DEBOUNCE_TIME  = 150;
static const unsigned long LINK_TIMEOUT_MS = 1200;

// ---- КОНФИГ ДЛЯ ПРИЕМА BMS ----
static const int BMS_ENOW_LEN = 4;      
static const uint8_t ENOW_MAGIC = 0xB1; 

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);

enum : uint8_t { MSG_CMD = 0xC1, MSG_STATUS = 0x5A };

struct CommandPacket {
  uint8_t type;
  uint8_t groupId;
  char password[8];
  uint8_t dstMac[6];
  float pulseWidthMs;
};

struct StatusPacket {
  uint8_t type;
  uint8_t groupId;
  char password[8];
  float currentPwmMs;
};

volatile uint32_t lastStatusMs = 0;
volatile float slavePwmMs = MIN_PWM;

volatile int packVoltageV = -1;
volatile int packCurrentA = -1;

uint8_t slaveMac[6] = {0};
bool slaveKnown = false;

int level = 1;
float pulseWidthMs = MIN_PWM;

unsigned long lastSend = 0;
unsigned long lastBtnTime = 0;

bool lastPlus = false;
bool lastMinus = false;
bool lastStart = false;

// ---------------- RX CALLBACK (Поддержка ядра 2.x / 3.x) ----------------
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onRx(const esp_now_recv_info_t * recvInfo, const uint8_t *data, int len) {
  const uint8_t* mac = recvInfo->src_addr;
#else
void onRx(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
#endif

  // ---- РАЗБОР BMS BROADCAST ----
  if (len == BMS_ENOW_LEN && data[0] == ENOW_MAGIC) {
    uint16_t v_raw = data[1] | (data[2] << 8);
    uint8_t i_raw = data[3];

    if (v_raw == 0 && i_raw == 0) {
      packVoltageV = -1;
      packCurrentA = -1;
    } else {
      packVoltageV = constrain((int)v_raw, 0, 99);
      packCurrentA = constrain((int)i_raw, 0, 250);
    }
    return;
  }

  // ---- SLAVE STATUS ----
  if (len != sizeof(StatusPacket)) return;

  StatusPacket st{};
  memcpy(&st, data, sizeof(st));

  if (st.type != MSG_STATUS) return;
  if (st.groupId != LINK_GROUP_ID) return;
  if (strncmp(st.password, SLAVE_PASSWORD, 6) != 0) return;

  if (!slaveKnown || memcmp(slaveMac, mac, 6) != 0) {
    memcpy(slaveMac, mac, 6);
    slaveKnown = true;

    if (!esp_now_is_peer_exist(slaveMac)) {
      esp_now_peer_info_t peer{};
      memset(&peer, 0, sizeof(peer));
      memcpy(peer.peer_addr, slaveMac, 6);
      peer.channel = 6; 
      peer.encrypt = false;
      peer.ifidx = WIFI_IF_STA;
      esp_now_add_peer(&peer);
    }
  }

  slavePwmMs = st.currentPwmMs;
  lastStatusMs = millis();
}

// ---------------- UI ----------------
static inline float levelToPwmMs(int lvl) {
  if (lvl < 1) lvl = 1;
  if (lvl > 6) lvl = 6;
  return 1.0f + 0.2f * (lvl - 1);
}

void drawDigit(int lvl, bool isStart) {
  bool linkUp = (millis() - lastStatusMs) <= LINK_TIMEOUT_MS;
  bool mismatch = linkUp && (fabsf(slavePwmMs - pulseWidthMs) > 0.05f);

  char ub[8], ib[8]; 

  // Напряжение и ток без десятичных разделителей
  if (packVoltageV < 0) snprintf(ub, sizeof(ub), "--");
  else snprintf(ub, sizeof(ub), "%d", packVoltageV);

  if (packCurrentA < 0) snprintf(ib, sizeof(ib), "--");
  else snprintf(ib, sizeof(ib), "%d", packCurrentA);

  u8g2.clearBuffer();

  // Статусная строка
  u8g2.setFont(u8g2_font_10x20_tr);
  u8g2.drawStr(0, 12, isStart ? "D" : "S");
  u8g2.drawStr(108, 12, linkUp ? "L" : "U");
  if (mismatch) u8g2.drawStr(92, 12, "!");

  // Центральная большая цифра уровня (высота 30px)
  u8g2.setFont(u8g2_font_fub30_tn);
  char s[2] = { char('0' + lvl), 0 };
  int dw = u8g2.getStrWidth(s);
  int dx = (128 - dw) / 2;
  u8g2.drawStr(dx, 56, s);

  // --- ОГРОМНЫЙ ШРИФТ ДЛЯ НАПРЯЖЕНИЯ И ТОКА ---
  u8g2.setFont(u8g2_font_fub20_tn); // Высота 20px, жирный, отлично виден на солнце
  int uw = u8g2.getStrWidth(ub);
  
  // Позиционирование напряжения слева от центра
  int ux = dx - 5 - uw; 
  if (ux < 0) ux = 0; 

  u8g2.drawStr(ux, 52, ub);
  u8g2.drawStr(dx + dw + 5, 52, ib); // Вывод тока справа от центра

  u8g2.sendBuffer();
}

static inline void refreshDisplay(bool start) {
  drawDigit(level, start);
}

// ---------------- SETUP ----------------
void setup() {
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_PLUS,  INPUT_PULLUP);
  pinMode(BTN_MINUS, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE); 
  esp_wifi_set_promiscuous(false);
  (void)esp_wifi_start();

  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Оптимизация под близкое расстояние (1 метр)
  WiFi.setTxPower(WIFI_POWER_7dBm);
  esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

  if (esp_now_init() != ESP_OK) while (true) delay(1000);
  esp_now_register_recv_cb(onRx);

  esp_now_peer_info_t peer{};
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, broadcastMAC, 6);
  peer.channel = 6; 
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  (void)esp_now_add_peer(&peer);

  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  refreshDisplay(false);
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long now = millis();

  bool start = (digitalRead(BTN_START) == LOW);
  bool plus  = (digitalRead(BTN_PLUS)  == LOW);
  bool minus = (digitalRead(BTN_MINUS) == LOW);

  if (now - lastBtnTime >= DEBOUNCE_TIME) {
    bool changed = false;

    if (plus && !lastPlus) { if (level < 6) level++; changed = true; lastBtnTime = now; }
    if (minus && !lastMinus) { if (level > 1) level--; changed = true; lastBtnTime = now; }

    if (changed || (start != lastStart)) refreshDisplay(start);
  }

  pulseWidthMs = start ? levelToPwmMs(level) : MIN_PWM;

  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;

    CommandPacket pkt{};
    pkt.type = MSG_CMD;
    pkt.groupId = LINK_GROUP_ID;
    strncpy(pkt.password, MASTER_PASSWORD, sizeof(pkt.password)-1);

    if (slaveKnown) memcpy(pkt.dstMac, slaveMac, 6);
    else memset(pkt.dstMac, 0, 6);

    pkt.pulseWidthMs = pulseWidthMs;

    const uint8_t* target = slaveKnown ? slaveMac : broadcastMAC;
    esp_now_send(target, (uint8_t*)&pkt, sizeof(pkt));

    refreshDisplay(start);
  }

  lastStart = start;
  lastPlus = plus;
  lastMinus = minus;
}
