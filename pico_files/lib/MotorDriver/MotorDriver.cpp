#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

static constexpr uint32_t SERIAL_BAUD = 9600;

// -------------------- SPI1 pins --------------------
// Если реально используешь GP10/11/12:
static constexpr uint8_t PIN_SPI1_SCK = 10;  // GP10 = SPI1 SCK
static constexpr uint8_t PIN_SPI1_MOSI = 11; // GP11 = SPI1 MOSI
static constexpr uint8_t PIN_SPI1_MISO = 8;  // GP12 = SPI1 MISO

// MCP2515 control pins
static constexpr uint8_t PIN_CAN_CS = 9;
static constexpr uint8_t PIN_CAN_INT = 7; // опционально

// CAN params
static constexpr uint8_t MCP2515_CLOCK = MCP_8MHZ;   // MCP_8MHZ или MCP_16MHZ
static constexpr uint8_t CAN_BITRATE = CAN_1000KBPS; // или CAN_500KBPS

// -------------------- Motor IDs --------------------
static constexpr uint8_t MOTOR_ID_A = 1;
static constexpr uint8_t MOTOR_ID_B = 2;
static constexpr uint8_t MOTOR_ID_C = 3;
static constexpr uint8_t MOTOR_ID_D = 4;

static constexpr uint16_t CAN_ID_BASE = 0x140;

// -------------------- SPI1 instance --------------------
// порядок: (miso, mosi, sck)
arduino::MbedSPI SPI1(PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);

// MCP2515 over SPI1
MCP_CAN CAN0(&SPI1, PIN_CAN_CS);

// moving params
const int motor_maxSpeed = 100;

static byte sendFrame(uint16_t canId, const uint8_t data[8])
{
    return CAN0.sendMsgBuf(canId, 0, 8, (uint8_t *)data);
}

static void enableMotor(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x88;

    uint16_t canId = CAN_ID_BASE + id;
    byte rc = sendFrame(canId, data);

    // Serial.print("ENABLE ID=");
    // Serial.print(id);
    // Serial.print(" rc=");
    // Serial.println(rc);
}

// speed control: 0xA2, speed * 100, int32 little-endian
static void motor(uint8_t id, int32_t speed)
{
    int32_t sp = speed * 1500;
    uint16_t canId = CAN_ID_BASE + id;

    uint8_t data[8];
    data[0] = 0xA2;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = (uint8_t)(sp & 0xFF);
    data[5] = (uint8_t)((sp >> 8) & 0xFF);
    data[6] = (uint8_t)((sp >> 16) & 0xFF);
    data[7] = (uint8_t)((sp >> 24) & 0xFF);

    byte rc = sendFrame(canId, data);
    if (rc != CAN_OK)
    {
        // Serial.print("motor(ID=");
        // Serial.print(id);
        // Serial.print(") send rc=");
        // Serial.println(rc);
    }
}

void motor_init()
{
    // Serial.begin(SERIAL_BAUD);

    pinMode(PIN_CAN_CS, OUTPUT);
    digitalWrite(PIN_CAN_CS, HIGH);

    if (PIN_CAN_INT != 255)
    {
        pinMode(PIN_CAN_INT, INPUT_PULLUP);
    }

    SPI1.begin();

    // Serial.println("Init MCP2515...");
    byte rc = CAN0.begin(MCP_ANY, CAN_BITRATE, MCP2515_CLOCK);
    // Serial.print("CAN0.begin rc=");
    // Serial.println(rc);

    if (rc != CAN_OK)
    {
        // Serial.println("MCP2515 init FAIL");
        while (1)
            delay(1000);
    }

    CAN0.setMode(MCP_NORMAL);
    // Serial.println("CAN NORMAL mode");

    // Включаем все 4 мотора
    enableMotor(MOTOR_ID_A);
    delay(10);
    enableMotor(MOTOR_ID_B);
    delay(10);
    enableMotor(MOTOR_ID_C);
    delay(10);
    enableMotor(MOTOR_ID_D);
    delay(10);
}

void move(int32_t spA, int32_t spB, int32_t spC, int32_t spD)
{
    motor(MOTOR_ID_A, constrain(spA, -motor_maxSpeed, motor_maxSpeed));
    motor(MOTOR_ID_B, constrain(spB, -motor_maxSpeed, motor_maxSpeed));
    motor(MOTOR_ID_C, constrain(spC, -motor_maxSpeed, motor_maxSpeed));
    motor(MOTOR_ID_D, constrain(spD, -motor_maxSpeed, motor_maxSpeed));
}

void move_angle(float ang, int speed, int u)
{
  int pA = speed * sin(radians(ang - 62.5));
  int pB = speed * sin(radians(ang - 117.5));
  int pC = speed * sin(radians(ang + 117.5));
  int pD = speed * sin(radians(ang + 62.5));

  move(pA + u, pB + u, pC + u, pD + u);
}

void stop() { move(0, 0, 0, 0); }

// -------------------- Helpers --------------------
static void printFrame(uint32_t rawId, uint8_t len, const uint8_t *data)
{
    const bool isExt = (rawId & 0x80000000UL) == 0x80000000UL;
    uint32_t canId = isExt ? (rawId & 0x1FFFFFFFUL) : (rawId & 0x7FFUL);

    Serial.print(isExt ? "EXT" : "STD");
    Serial.print(" ID=0x");
    Serial.print(canId, HEX);
    Serial.print(" DLC=");
    Serial.print(len);
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


// Запрос статуса 0x9A
static void requestStatus(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x9A;

    uint16_t canId = CAN_ID_BASE + id;
    byte rc = sendFrame(canId, data);

    // Serial.print("STATUS ID=");
    // Serial.print(id);
    // Serial.print(" rc=");
    // Serial.println(rc);
}