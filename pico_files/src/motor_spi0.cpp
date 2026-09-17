/*
  MF4015v2 (RMD-class) + Raspberry Pi Pico (RP2040) + MCP2515 over SPI0
  Переписано под SPI0 (объект SPI) без MbedSPI.

  ВАЖНО:
  - Здесь используется framework-arduino-mbed (PlatformIO pico).
  - В этом core чаще всего доступен только SPI (SPI0).
  - setSCK/setTX/setRX на RP2040 mbed-core могут быть недоступны/не нужны.
    Поэтому используем дефолтные пины SPI0 Pico, либо просто SPI.begin().

  Дефолтные пины SPI0 для Pico (часто используемые):
    SCK  = GP18
    MOSI = GP19
    MISO = GP16
  CS и INT — любые GPIO (CS обязательно).

  Если у тебя проводами подключено к другим GPIO — проще переподключить на эти дефолтные.
  (Иначе придётся уходить на другое ядро, где есть pin remap через setSCK/setTX/setRX.)
*/

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

// -------------------- Настройки UART --------------------
static constexpr uint32_t SERIAL_BAUD = 115200;

// -------------------- Пины Pico (SPI0, дефолт) --------------------
static constexpr uint8_t PIN_SPI0_SCK = 18;  // GP18 = SPI0 SCK
static constexpr uint8_t PIN_SPI0_MOSI = 19; // GP19 = SPI0 MOSI (TX)
static constexpr uint8_t PIN_SPI0_MISO = 16; // GP16 = SPI0 MISO (RX)

static constexpr uint8_t PIN_CAN_CS = 17; // CS для MCP2515 (любой GPIO)
static constexpr uint8_t PIN_CAN_INT = 7; // /INT (active-low). Если не подключён — поставьте 255

static constexpr bool USE_CAN_INT = false; // false, если INT не подключён

// -------------------- Параметры MCP2515 --------------------
// Проверь кварц на модуле: MCP_8MHZ или MCP_16MHZ
static constexpr uint8_t MCP2515_CLOCK = MCP_8MHZ;

// Скорость CAN (в примере 1 Mbps, как у тебя было)
static constexpr uint8_t CAN_BITRATE = CAN_1000KBPS;

static constexpr uint8_t CAN_IDMODE = MCP_ANY;

// -------------------- Параметры мотора --------------------
static constexpr uint8_t MOTOR_ID = 1;
static constexpr uint16_t CAN_ID_BASE = 0x140;
static constexpr uint16_t CAN_ID_MOTOR = CAN_ID_BASE + MOTOR_ID;

static constexpr uint8_t DLC_8 = 8;

// MCP_CAN на SPI0 (SPI)
MCP_CAN CAN0(&SPI, PIN_CAN_CS);

// -------------------- Вспомогательные функции --------------------
static void printFrame(uint32_t rawId, uint8_t len, const uint8_t *data)
{
  const bool isExt = (rawId & 0x80000000UL) == 0x80000000UL;
  const bool isRtr = (rawId & 0x40000000UL) == 0x40000000UL;

  uint32_t canId = rawId;
  if (isExt)
    canId = rawId & 0x1FFFFFFFUL;
  else
    canId = rawId & 0x7FFUL;

  Serial.print(isExt ? "EXT" : "STD");
  Serial.print(" ID=0x");
  Serial.print(canId, HEX);
  Serial.print(" DLC=");
  Serial.print(len);
  if (isRtr)
    Serial.print(" RTR");

  Serial.print(" DATA=");
  for (uint8_t i = 0; i < len; i++)
  {
    if (data[i] < 0x10)
      Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

static bool canSendStd(uint16_t stdId, const uint8_t data[DLC_8])
{
  const uint8_t ext = 0; // стандартный кадр
  const uint8_t rc = CAN0.sendMsgBuf((uint32_t)stdId, ext, DLC_8, (uint8_t *)data);
  if (rc != CAN_OK)
  {
    Serial.print("sendMsgBuf failed, rc=");
    Serial.println(rc);
    return false;
  }
  return true;
}

// static bool canReadOne(uint32_t &outId, uint8_t &outLen, uint8_t outData[8])
// {
//   if (USE_CAN_INT && PIN_CAN_INT != 255)
//   {
//     if (digitalRead(PIN_CAN_INT) != LOW)
//       return false;
//   }
//   else
//   {
//     if (CAN0.checkReceive() != CAN_MSGAVAIL)
//       return false;
//   }

//   outLen = 0;
//   const uint8_t rc = CAN0.readMsgBuf(&outId, &outLen, outData);
//   if (rc != CAN_OK)
//   {
//     Serial.print("readMsgBuf failed, rc=");
//     Serial.println(rc);
//     return false;
//   }
//   return true;
// }

static bool canReadOne(uint32_t &outId, uint8_t &outLen, uint8_t outData[8])
{
  if (CAN0.checkReceive() != CAN_MSGAVAIL)
    return false;
  outLen = 0;
  return (CAN0.readMsgBuf(&outId, &outLen, outData) == CAN_OK);
}

static bool waitEcho(uint16_t expectId, const uint8_t expectData[DLC_8], uint32_t timeoutMs)
{
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs)
  {
    uint32_t rxId = 0;
    uint8_t rxLen = 0;
    uint8_t rxBuf[8] = {0};

    if (canReadOne(rxId, rxLen, rxBuf))
    {
      const bool isExt = (rxId & 0x80000000UL) == 0x80000000UL;
      uint32_t canId = isExt ? (rxId & 0x1FFFFFFFUL) : (rxId & 0x7FFUL);

      if (!isExt && canId == expectId && rxLen == DLC_8)
      {
        bool same = true;
        for (uint8_t i = 0; i < DLC_8; i++)
        {
          if (rxBuf[i] != expectData[i])
          {
            same = false;
            break;
          }
        }
        Serial.print("RX: ");
        printFrame(rxId, rxLen, rxBuf);
        if (same)
          return true;
      }
      else
      {
        Serial.print("RX(other): ");
        printFrame(rxId, rxLen, rxBuf);
      }
    }
    delay(2);
  }
  return false;
}

static bool requestStatus9A(int8_t &tempC, float &voltageV, uint16_t &errFlags)
{
  uint8_t tx[8] = {0};
  tx[0] = 0x9A;

  Serial.println("TX: status 0x9A ...");
  if (!canSendStd(CAN_ID_MOTOR, tx))
    return false;

  const uint32_t t0 = millis();
  while ((millis() - t0) < 200)
  {
    uint32_t rxId = 0;
    uint8_t rxLen = 0;
    uint8_t rxBuf[8] = {0};

    if (canReadOne(rxId, rxLen, rxBuf))
    {
      const bool isExt = (rxId & 0x80000000UL) == 0x80000000UL;
      uint32_t canId = isExt ? (rxId & 0x1FFFFFFFUL) : (rxId & 0x7FFUL);

      if (!isExt && canId == CAN_ID_MOTOR && rxLen == DLC_8 && rxBuf[0] == 0x9A)
      {
        tempC = (int8_t)rxBuf[1];

        const uint16_t vRaw = (uint16_t)rxBuf[4] | ((uint16_t)rxBuf[5] << 8);
        voltageV = (float)vRaw * 0.1f;

        errFlags = (uint16_t)rxBuf[6] | ((uint16_t)rxBuf[7] << 8);

        Serial.print("RX: ");
        printFrame(rxId, rxLen, rxBuf);
        return true;
      }
      else
      {
        Serial.print("RX(other): ");
        printFrame(rxId, rxLen, rxBuf);
      }
    }
    delay(2);
  }
  return false;
}

static void sendSpeedRPM(int16_t rpm)
{
  // Команда speed control (часто 0xA2 у RMD, но у тебя мы пробуем 0xA2 как speed)
  // ВАЖНО: у многих RMD скорость в [4..5] little-endian.
  uint8_t d[8] = {0};
  d[0] = 0xA2; // <-- speed control у многих RMD. Если у тебя другой протокол — поменяем.

  d[4] = (uint8_t)(rpm & 0xFF);
  d[5] = (uint8_t)((rpm >> 8) & 0xFF);

  byte s = CAN0.sendMsgBuf(CAN_ID_MOTOR, 0, 8, d);
  if (s != CAN_OK)
  {
    Serial.print("SEND A2 rc=");
    Serial.println(s);
  }
}

static void motor(uint8_t ID, int32_t speed)
{
  int32_t sp = speed * 100; // масштабирование как в твоём коде

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
  if (rc != CAN_OK)
  {
    Serial.print("motor() send rc=");
    Serial.println(rc);
  }
}

// -------------------- setup/loop --------------------
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println();
  Serial.println("=== Pico + MCP2515 (SPI0) + MF4015v2 CAN demo ===");

  // CS и INT
  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);

  if (USE_CAN_INT && PIN_CAN_INT != 255)
  {
    pinMode(PIN_CAN_INT, INPUT_PULLUP);
  }

  // SPI0
  // В Arduino-mbed для Pico обычно достаточно просто SPI.begin(),
  // а пины берутся дефолтные (GP18/19/16).
  SPI.begin();

  // Если у тебя проводами реально подключены GP18/19/16 — всё ок.
  // Если подключено к другим — лучше переподключить к дефолтным пинам SPI0.

  Serial.println("Init MCP2515...");
  const uint8_t rc = CAN0.begin(CAN_IDMODE, CAN_BITRATE, MCP2515_CLOCK);
  if (rc == CAN_OK)
  {
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else
  {
    Serial.print("Error Initializing MCP2515, rc=");
    Serial.println(rc);
    Serial.println("Check: SPI wiring (SPI0 pins), CS pin, MCP2515 crystal (8/16MHz), power/levels.");
    while (true)
      delay(1000);
  }

  // Фильтр на мотор (STD ID)
  if (CAN0.init_Mask(0, 0, 0x7FF) == CAN_OK &&
      CAN0.init_Filt(0, 0, CAN_ID_MOTOR) == CAN_OK)
  {
    Serial.println("Mask/Filter set for motor ID.");
  }
  else
  {
    Serial.println("Warning: cannot set mask/filter (continuing).");
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("CAN mode set to NORMAL.");

  // 1) "Motor running" (0x88)
  uint8_t enableCmd[8] = {0};
  enableCmd[0] = 0x88;
  byte s_en = CAN0.sendMsgBuf(CAN_ID_MOTOR, 0, 8, enableCmd);
  Serial.print("ENABLE rc=");
  Serial.println(s_en);

  Serial.print("TX: motor-running 0x88 to CAN ID=0x");
  Serial.println(CAN_ID_MOTOR, HEX);

  if (!canSendStd(CAN_ID_MOTOR, enableCmd))
  {
    Serial.println("Enable send failed.");
    return;
  }

  // 2) Ждём эхо
  if (waitEcho(CAN_ID_MOTOR, enableCmd, 200))
  {
    Serial.println("OK: motor echoed 0x88 (link looks alive).");
  }
  else
  {
    Serial.println("WARN: no echo for 0x88 within timeout.");
    Serial.println("If nothing responds: try CAN_500KBPS vs CAN_1000KBPS and/or MCP_16MHZ.");
  }

  // 3) Статус 0x9A
  int8_t tempC = 0;
  float voltageV = 0;
  uint16_t errFlags = 0;
  if (requestStatus9A(tempC, voltageV, errFlags))
  {
    Serial.print("Status: T=");
    Serial.print(tempC);
    Serial.print("C, V=");
    Serial.print(voltageV, 1);
    Serial.print("V, err=0x");
    Serial.println(errFlags, HEX);
  }
  else
  {
    Serial.println("WARN: status 0x9A no reply.");
  }
}

void loop()
{
  // 1) постоянно задаём скорость (каждые 20 мс)
  static uint32_t t_spd = 0;
  if (millis() - t_spd >= 20)
  {
    t_spd = millis();

    motor(MOTOR_ID, 500); // <-- Поставь 50..300 для теста. Отрицательное = в обратную сторону.
  }

  // 2) раз в секунду запрос статуса 0x9A (по желанию)
  static uint32_t t_stat = 0;
  if (millis() - t_stat >= 1000)
  {
    t_stat = millis();
    uint16_t canId = CAN_ID_BASE + MOTOR_ID;
    uint8_t st[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};
    byte rc = CAN0.sendMsgBuf(canId, 0, 8, st);
    Serial.print("SEND 9A rc=");
    Serial.println(rc);
  }

  // 3) читаем входящие
  uint32_t rxId = 0;
  uint8_t rxLen = 0;
  uint8_t rxBuf[8] = {0};

  if (CAN0.checkReceive() == CAN_MSGAVAIL)
  {
    if (CAN0.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK)
    {
      // Чтобы не спамить, можно печатать только интересные ответы:
      if (rxLen == 8 && (rxBuf[0] == 0x9A || rxBuf[0] == 0xA2 || rxBuf[0] == 0x88))
      {
        Serial.print("RX: ");
        printFrame(rxId, rxLen, rxBuf);
      }
    }
  }

  delay(1);
}