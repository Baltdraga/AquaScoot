// ===== TM =====/espressif/arduino-esp32/2.0.17/NimBLE/1.4.1
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <NimBLEDevice.h>
#include <cmath>
#include <cstring>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ---------------- CONFIG ----------------
static const NimBLEAddress BMS_ADDR("78:7f:74:bc:1f:c6");

static NimBLEUUID UUID_SVC((uint16_t)0xFFE0);
static NimBLEUUID UUID_NOTIFY((uint16_t)0xFFE1);
static NimBLEUUID UUID_WRITE((uint16_t)0xFFE2);

static const uint8_t POLL_CMD[] = {
    0x7E, 0xA1, 0x01, 0x00, 0x00, 0xBE, 0x18, 0x55, 0xAA, 0x55};

static const int V_OFF = 78;
static const int I_OFF = 80;

struct ESPNowData {
  float voltage;
  float current;
};

static ESPNowData espData = {NAN, NAN};
static const uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t WIFI_CHANNEL = 6;
static const uint8_t ENOW_MAGIC = 0xB1;

static uint8_t rxbuf[512];
static size_t rxLen = 0;
static volatile bool sPendingEspNow = false;

// ---------------- OLED ----------------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ---------------- BLE ----------------
static NimBLEClient* client = nullptr;
static NimBLERemoteCharacteristic* chrN = nullptr;
static NimBLERemoteCharacteristic* chrW = nullptr;

// ---------------- UTILS ----------------
static inline uint16_t u16le(const uint8_t* p, size_t o) {
  return (uint16_t)p[o] | ((uint16_t)p[o + 1] << 8);
}

static void drawOLED() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_10x20_tf);

  oled.setCursor(0, 24);
  oled.print("U ");
  isnan(espData.voltage) ? oled.print("--.--") : oled.print(espData.voltage, 2);

  oled.setCursor(0, 54);
  oled.print("I ");
  isnan(espData.current) ? oled.print("-.-") : oled.print(espData.current, 1);

  oled.sendBuffer();
}

// Вспомогательная функция для сборки и отправки пакета по ESP-NOW
static void sendEspNowPacket(float u, float i) {
  uint16_t vr = 0;
  uint8_t ti = 0;

  // Если значения валидны — пакуем их, иначе отправляем 0 (сигнал прочерков для ресивера)
  if (!isnan(u) && !isnan(i)) {
    vr = (uint16_t)constrain((int)lroundf(u), 0, 65535);
    ti = (uint8_t)constrain((int)lroundf(i), 0, 255);
  }

  const uint8_t tx[4] = {
      ENOW_MAGIC,
      (uint8_t)(vr & 0xFF),
      (uint8_t)(vr >> 8),
      ti};
  (void)esp_now_send(broadcastMac, tx, sizeof(tx));
}

// ---------------- PARSER ----------------
static bool parseFrame(const uint8_t* f, size_t n) {
  if (n < 8) return false;
  if (f[0] != 0x7E) return false; // Исправлено: добавлены квадратные скобки
  if (!(f[n - 2] == 0xAA && f[n - 1] == 0x55)) return false;

  const size_t end = n - 4;
  if (V_OFF + 1 >= end || I_OFF + 1 >= end) return false;

  espData.voltage = u16le(f, V_OFF) * 0.01f;
  espData.current = u16le(f, I_OFF) * 0.1f;
  drawOLED();
  return true;
}

static void alignTo7E() {
  size_t i = 0;
  while (i < rxLen && rxbuf[i] != 0x7E) i++;
  if (i == 0) return;
  if (i >= rxLen) {
    rxLen = 0;
    return;
  }
  memmove(rxbuf, rxbuf + i, rxLen - i);
  rxLen -= i;
}

static void processRxBuffer() {
  for (;;) {
    alignTo7E();
    if (rxLen < 4) break;

    int endPos = -1;
    for (size_t i = 0; i + 1 < rxLen; i++) {
      if (rxbuf[i] == 0xAA && rxbuf[i + 1] == 0x55) {
        endPos = (int)(i + 1);
        break;
      }
    }
    if (endPos < 0) break;

    const size_t frameLen = (size_t)endPos + 1;
    if (parseFrame(rxbuf, frameLen)) sPendingEspNow = true;

    memmove(rxbuf, rxbuf + frameLen, rxLen - frameLen);
    rxLen -= frameLen;
  }
}

// ---------------- NOTIFY ----------------
static void notifyCB(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  if (!len) return;
  if (rxLen + len > sizeof(rxbuf)) rxLen = 0;
  memcpy(rxbuf + rxLen, data, len);
  rxLen += len;
  processRxBuffer();
}

// ---------------- BLE CONNECT ----------------
static bool finishGattSubscribe() {
  NimBLERemoteService* svc = client->getService(UUID_SVC);
  if (!svc) return false;

  chrN = svc->getCharacteristic(UUID_NOTIFY);
  chrW = svc->getCharacteristic(UUID_WRITE);
  if (!chrN || !chrW) return false;

  if (!chrN->subscribe(true, notifyCB)) return false;

  chrW->writeValue(POLL_CMD, sizeof(POLL_CMD), false);

  return true;
}

static bool connectFast() {
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setConnectTimeout(10);
  }

  if (client->isConnected()) client->disconnect();

  if (!client->connect(BMS_ADDR)) return false;

  return finishGattSubscribe();
}

// ---------------- ESP-NOW SETUP ----------------
static void initESPNow() {
  WiFi.mode(WIFI_STA);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return;

  WiFi.setTxPower(WIFI_POWER_7dBm);
  esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastMac, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

// ---------------- SETUP ----------------
void setup() {
  delay(200);

  Wire.begin(8, 9);
  oled.begin();
  drawOLED();

  initESPNow();

  NimBLEDevice::init("");
  NimBLEDevice::setMTU(247);

  connectFast();
}

// ---------------- LOOP ----------------
void loop() {
  const uint32_t now = millis();

  // Отправка по прилету свежих данных от BLE
  if (sPendingEspNow) {
    sPendingEspNow = false;
    sendEspNowPacket(espData.voltage, espData.current);
  }

  // Если BLE отключен — сбрасываем данные в NAN и отправляем ресиверу пакет с нулями каждые 500 мс
  if (!client || !client->isConnected()) {
    espData.voltage = NAN;
    espData.current = NAN;
    rxLen = 0;
    drawOLED();

    static uint32_t lastDisconnectSend = 0;
    if (now - lastDisconnectSend >= 500) {
      lastDisconnectSend = now;
      sendEspNowPacket(NAN, NAN); // Принудительно шлем "прочерки" в эфир
    }

    delay(500);
    connectFast();
    return;
  }

  static uint32_t lastPoll = 0;
  if (now - lastPoll >= 1000) {
    lastPoll = now;
    if (chrW) chrW->writeValue(POLL_CMD, sizeof(POLL_CMD), false);
  }

  delay(20);
}
