#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

static constexpr uint32_t SERIAL_BAUD = 115200;

// -------------------- SPI1 pins (ПРОВЕРЬ ПРОВОДА) --------------------
// Типовые аппаратные пины SPI1 на Raspberry Pi Pico:
static constexpr uint8_t PIN_SPI1_SCK  = 10; // GP10 = SPI1 SCK
static constexpr uint8_t PIN_SPI1_MOSI = 11; // GP11 = SPI1 MOSI (TX)
static constexpr uint8_t PIN_SPI1_MISO = 8; // GP12 = SPI1 MISO (RX)

// MCP2515 control pins
static constexpr uint8_t PIN_CAN_CS  = 9;  // любой GPIO -> CS
static constexpr uint8_t PIN_CAN_INT = 7;  // любой GPIO -> INT (опционально)

// CAN params
static constexpr uint8_t MCP2515_CLOCK = MCP_8MHZ;      // MCP_8MHZ или MCP_16MHZ
static constexpr uint8_t CAN_BITRATE   = CAN_1000KBPS;  // или CAN_500KBPS

// Motor params
static constexpr uint8_t  MOTOR_ID     = 3;
static constexpr uint16_t CAN_ID_BASE  = 0x140;

// -------------------- SPI1 instance (arduino-mbed) --------------------
// ВАЖНО: порядок в MbedSPI: (miso, mosi, sck)
arduino::MbedSPI SPI1(PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);

// MCP2515 over SPI1
MCP_CAN CAN0(&SPI1, PIN_CAN_CS);

// -------------------- Helpers --------------------
static void printFrame(uint32_t rawId, uint8_t len, const uint8_t *data) {
  const bool isExt = (rawId & 0x80000000UL) == 0x80000000UL;
  uint32_t canId = isExt ? (rawId & 0x1FFFFFFFUL) : (rawId & 0x7FFUL);

  Serial.print(isExt ? "EXT" : "STD");
  Serial.print(" ID=0x");
  Serial.print(canId, HEX);
  Serial.print(" DLC=");
  Serial.print(len);
  Serial.print(" DATA=");
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

/*
  Speed control (как в твоей функции):
  - CAN ID = 0x140 + ID
  - cmd = 0xA2
  - speed кодируется int32 little-endian, scale = speed*100
*/
static void motor(uint8_t ID, int32_t speed) {
  int32_t sp = speed * 100;

  uint16_t canId = CAN_ID_BASE + ID;

  uint8_t data[8];
  data[0] = 0xA2;
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = (uint8_t)(sp & 0xFF);
  data[5] = (uint8_t)((sp >> 8) & 0xFF);
  data[6] = (uint8_t)((sp >> 16) & 0xFF);
  data[7] = (uint8_t)((sp >> 24) & 0xFF);

  byte rc = CAN0.sendMsgBuf(canId, 0, 8, data);
  if (rc != CAN_OK) {
    Serial.print("motor() send rc=");
    Serial.println(rc);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);

  if (PIN_CAN_INT != 255) {
    pinMode(PIN_CAN_INT, INPUT_PULLUP);
  }

  // Запускаем SPI1 (пины уже заданы конструктором MbedSPI)
  SPI1.begin();

  Serial.println("Init MCP2515...");
  byte rc = CAN0.begin(MCP_ANY, CAN_BITRATE, MCP2515_CLOCK);
  Serial.print("CAN0.begin rc=");
  Serial.println(rc);

  if (rc != CAN_OK) {
    Serial.println("MCP2515 init FAIL (check wiring/CS/clock/bitrate).");
    while (1) delay(1000);
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("CAN NORMAL mode");

  // enable motor (0x88)
  uint16_t canId = CAN_ID_BASE + MOTOR_ID;
  uint8_t en[8] = {0};
  en[0] = 0x88;
  byte rc_en = CAN0.sendMsgBuf(canId, 0, 8, en);
  Serial.print("SEND 88 rc=");
  Serial.println(rc_en);
}

void loop() {
  // 1) Постоянно задаём скорость (каждые 20 мс)
  static uint32_t t_spd = 0;
  if (millis() - t_spd >= 20) {
    t_spd = millis();
    motor(MOTOR_ID, -400);   // 50..300 тест; отрицательное = назад
  }

  // 2) Раз в секунду запрос статуса 0x9A
  static uint32_t t_stat = 0;
  if (millis() - t_stat >= 1000) {
    t_stat = millis();
    uint16_t canId = CAN_ID_BASE + MOTOR_ID;
    uint8_t st[8] = {0x9A,0,0,0,0,0,0,0};
    byte s = CAN0.sendMsgBuf(canId, 0, 8, st);
    Serial.print("SEND 9A rc=");
    Serial.println(s);
  }

  // 3) Читаем входящие
  uint32_t rxId = 0;
  uint8_t rxLen = 0;
  uint8_t rxBuf[8] = {0};

  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    if (CAN0.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
      if (rxLen == 8 && (rxBuf[0] == 0x9A || rxBuf[0] == 0xA2 || rxBuf[0] == 0x88)) {
        Serial.print("RX: ");
        printFrame(rxId, rxLen, rxBuf);
      }
    }
  }

  delay(1);
}