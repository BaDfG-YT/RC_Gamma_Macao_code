#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <MotorDriver.h>
#include <Regulators.h>

static constexpr uint32_t SERIAL_BAUD = 9600;

// -------------------- Waveshare RP2350-CAN: XL2515 on SPI1 --------------------
// Onboard wiring (fixed on the board):
//   SCK  -> GP10
//   MOSI -> GP11
//   MISO -> GP12   (NB: on the old RP2040 setup MISO was GP8 - here it's GP12)
//   CS   -> GP9
//   INT  -> GP8
static constexpr uint8_t PIN_CAN_SCK = 10;
static constexpr uint8_t PIN_CAN_MOSI = 11;
static constexpr uint8_t PIN_CAN_MISO = 12;
static constexpr uint8_t PIN_CAN_CS = 9;
static constexpr uint8_t PIN_CAN_INT = 8; // optional, for interrupt-driven reception

// CAN params
static constexpr uint8_t MCP2515_CLOCK = MCP_16MHZ;
static constexpr uint8_t CAN_BITRATE = CAN_1000KBPS;

// -------------------- Motor IDs (MF4015 v2, protocol 0x140 + ID) --------------------
static constexpr uint8_t MOTOR_ID_A = 3;
static constexpr uint8_t MOTOR_ID_B = 2;

static constexpr uint16_t CAN_ID_BASE = 0x140;

// -------------------- CAN instance --------------------
MCP_CAN CAN0(&SPI1, PIN_CAN_CS);

// -------------------- Directions --------------------
static constexpr int32_t DIR_A = -1;
static constexpr int32_t DIR_B = +1;

const int motor_maxSpeed = 200;

static byte sendFrame(uint16_t canId, const uint8_t data[8])
{
    return CAN0.sendMsgBuf(canId, 0, 8, (uint8_t *)data);
}

static void enableMotor(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x88;
    uint16_t canId = CAN_ID_BASE + id;
    sendFrame(canId, data);
}

static void motor(uint8_t id, int32_t speed)
{
    int32_t sp = speed * 2000;
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

    sendFrame(canId, data);
}

void motor_init()
{
    pinMode(PIN_CAN_CS, OUTPUT);
    digitalWrite(PIN_CAN_CS, HIGH);

    pinMode(PIN_CAN_INT, INPUT_PULLUP);

    SPI1.setSCK(PIN_CAN_SCK);
    SPI1.setTX(PIN_CAN_MOSI);
    SPI1.setRX(PIN_CAN_MISO);
    SPI1.begin();

    byte rc = CAN0.begin(MCP_ANY, CAN_BITRATE, MCP2515_CLOCK);
    if (rc != CAN_OK)
    {
        Serial.println("XL2515 init FAIL - motors disabled");
        return;
    }

    CAN0.setMode(MCP_NORMAL);

    enableMotor(MOTOR_ID_A);
    delay(10);
    enableMotor(MOTOR_ID_B);
    delay(10);
}

// ==================== MF4015 v2 encoders (0x90) ====================

static bool waitReply(uint8_t id, uint8_t cmd, uint8_t out[8],
                      unsigned long timeout_ms = 10)
{
    const uint32_t wantId = CAN_ID_BASE + id;
    unsigned long t0 = millis();

    while (millis() - t0 < timeout_ms)
    {
        if (CAN0.checkReceive() == CAN_MSGAVAIL)
        {
            uint32_t rxId;
            uint8_t len = 0;
            uint8_t buf[8];
            CAN0.readMsgBuf(&rxId, &len, buf);

            if ((rxId & 0x7FF) == wantId && len == 8 && buf[0] == cmd)
            {
                memcpy(out, buf, 8);
                return true;
            }
        }
    }
    return false;
}

static bool readEncoder(uint8_t id, EncoderState &st)
{
    st.valid = false;

    uint8_t req[8] = {0};
    req[0] = 0x90;
    if (sendFrame(CAN_ID_BASE + id, req) != CAN_OK)
        return false;

    uint8_t rep[8];
    if (!waitReply(id, 0x90, rep))
        return false;

    st.pos = (uint16_t)(rep[2] | (rep[3] << 8));
    st.raw = (uint16_t)(rep[4] | (rep[5] << 8));
    st.offset = (uint16_t)(rep[6] | (rep[7] << 8));
    st.valid = true;
    return true;
}

bool readEncoders(EncoderState &encA, EncoderState &encB)
{
    readEncoder(MOTOR_ID_A, encA);
    readEncoder(MOTOR_ID_B, encB);
    return encA.valid && encB.valid;
}

// ==================== Multi-turn angle (0x92) ====================

static bool readMultiTurnRaw(uint8_t id, int64_t &centideg)
{
    uint8_t req[8] = {0};
    req[0] = 0x92;
    if (sendFrame(CAN_ID_BASE + id, req) != CAN_OK)
        return false;

    uint8_t rep[8];
    if (!waitReply(id, 0x92, rep))
        return false;

    uint64_t v = 0;
    for (int i = 0; i < 7; i++)
        v |= ((uint64_t)rep[1 + i]) << (8 * i);
    if (v & 0x0080000000000000ULL)
        v |= 0xFF00000000000000ULL;

    centideg = (int64_t)v;
    return true;
}

bool readAngles(float &angA_deg, float &angB_deg)
{
    int64_t a = 0, b = 0;
    bool okA = readMultiTurnRaw(MOTOR_ID_A, a);
    bool okB = readMultiTurnRaw(MOTOR_ID_B, b);

    if (okA)
        angA_deg = DIR_A * (float)(a / 100.0);
    if (okB)
        angB_deg = DIR_B * (float)(b / 100.0);

    return okA && okB;
}

bool readTurns(float &turnsA, float &turnsB)
{
    float a, b;
    if (!readAngles(a, b))
        return false;
    turnsA = a / 360.0f;
    turnsB = b / 360.0f;
    return true;
}

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

static void requestStatus(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x9A;
    uint16_t canId = CAN_ID_BASE + id;
    sendFrame(canId, data);
}

// ==================== Movement ====================

void move(int32_t spA, int32_t spB)
{
    motor(MOTOR_ID_A, constrain(DIR_A * spA, -motor_maxSpeed, motor_maxSpeed));
    motor(MOTOR_ID_B, constrain(DIR_B * spB, -motor_maxSpeed, motor_maxSpeed));
}

void obsession(int32_t sp) { move(sp, -sp); }

// Turn off the motor. After 0x80 the power output is disabled,
// the shaft can spin freely.
static void motorOff(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x80;
    sendFrame(CAN_ID_BASE + id, data);
}

// Hold a given multi-turn angle (0xA4).
// Unlike 0xA5 (single-turn angle modulo + direction),
// here the angle is absolute - direction is never chosen,
// so there's no risk of moving the wrong way while locking.
static void holdPositionRaw(uint8_t id, int32_t angleCentideg, uint16_t maxSpeedDps = 15)
{
    uint8_t data[8] = {0};
    data[0] = 0xA4;

    data[2] = (uint8_t)(maxSpeedDps & 0xFF);
    data[3] = (uint8_t)((maxSpeedDps >> 8) & 0xFF);

    data[4] = (uint8_t)(angleCentideg & 0xFF);
    data[5] = (uint8_t)((angleCentideg >> 8) & 0xFF);
    data[6] = (uint8_t)((angleCentideg >> 16) & 0xFF);
    data[7] = (uint8_t)((angleCentideg >> 24) & 0xFF);

    sendFrame(CAN_ID_BASE + id, data);
}

// Lock one motor at its current (multi-turn) angle.
static bool lockAtCurrent(uint8_t id)
{
    int64_t ang = 0;
    if (!readMultiTurnRaw(id, ang))
        return false;

    holdPositionRaw(id, (int32_t)ang, 500);
    return true;
}

// Stop the motors.
// hold = true:  motors lock at their current position (0xA4).
// hold = false: motors turn off and can spin freely.
void stop(bool hold)
{
    move(0, 0);

    if (!hold)
    {
        delay(30);
        motorOff(MOTOR_ID_A);
        motorOff(MOTOR_ID_B);
        return;
    }

    delay(100);

    lockAtCurrent(MOTOR_ID_A);
    lockAtCurrent(MOTOR_ID_B);
}

// ==================== Movement by encoders ====================

// -------------------- Speed profile --------------------
static constexpr int V_MIN = 6;
static constexpr float ACCEL_DIST_DEG = 950.0f;
static constexpr float DECEL_DIST_DEG = 950.0f;

// -------------------- Straight-line hold (PID) --------------------
static PID pid_straight;
float straight_p = 0.15f;
float straight_i = 0.0f;
float straight_d = 0.5f;

// -------------------- Robot geometry --------------------
static constexpr float WHEEL_D_MM = 45.2f;
static constexpr float TRACK_MM = 134.5f;

bool move_deg(float deg, int speed, unsigned long timeout_ms, MoveProgressCb progress)
{
    float a0, b0;
    if (!readAngles(a0, b0))
        return false;

    pid_reset(pid_straight);

    int dir = (deg >= 0) ? 1 : -1;
    float target = fabsf(deg);
    unsigned long t0 = millis();
    unsigned long t_cb = 0;

    float pa = 0, pb = 0;
    bool ok = false;

    while (millis() - t0 < timeout_ms)
    {
        float a, b;
        if (!readAngles(a, b))
        {
            stop();
            return false;
        }

        pa = dir * (a - a0);
        pb = dir * (b - b0);
        float traveled = 0.5f * (pa + pb);

        if (traveled >= target)
        {
            ok = true;
            break;
        }

        float rest = target - traveled;
        float v_acc = V_MIN + (speed - V_MIN) * (traveled / ACCEL_DIST_DEG);
        float v_dec = V_MIN + (speed - V_MIN) * (rest / DECEL_DIST_DEG);
        int v = (int)min(min(v_acc, v_dec), (float)speed);
        v = max(v, V_MIN);

        // --- PID straight-line hold: e = how much A is ahead of B ---
        float e = pa - pb;
        int u = lround(pid_reg(pid_straight, e, straight_p, straight_i, straight_d));
        u = constrain(u, -v / 2, v / 2);

        move(dir * (v - u), dir * (v + u));

        if (progress && millis() - t_cb >= 100)
        {
            t_cb = millis();
            progress(pa, pb, target);
        }

        delay(2);
    }

    stop();
    if (progress)
        progress(pa, pb, target);
    return ok;
}

bool move_mm(float mm, int speed, unsigned long timeout_ms, MoveProgressCb progress)
{
    return move_deg(mm / (WHEEL_D_MM * PI) * 360, speed, timeout_ms, progress);
}

// Move mm millimeters with PID steering correction from an external error.
// error_src is called every cycle (e.g. the red-target deviation from the camera).
// Distance and speed profile come from encoders, heading comes from PID on the error.
bool move_mm_pid(float mm, ErrorCb error_src, float kp, float ki, float kd, int speed,
                 unsigned long timeout_ms, MoveProgressCb progress)
{
    float a0, b0;
    if (!readAngles(a0, b0))
        return false;

    static PID pid_ext; // dedicated PID so it doesn't interfere with pid_straight
    pid_reset(pid_ext);

    int dir = (mm >= 0) ? 1 : -1;
    float target = fabsf(mm) / (WHEEL_D_MM * PI) * 360.0f;

    unsigned long t0 = millis();
    unsigned long t_cb = 0;

    float pa = 0, pb = 0;
    bool ok = false;

    while (millis() - t0 < timeout_ms)
    {
        float a, b;
        if (!readAngles(a, b))
        {
            stop();
            return false;
        }

        pa = dir * (a - a0);
        pb = dir * (b - b0);
        float traveled = 0.5f * (pa + pb);

        if (traveled >= target)
        {
            ok = true;
            break;
        }

        float rest = target - traveled;
        float v_acc = V_MIN + (speed - V_MIN) * (traveled / ACCEL_DIST_DEG);
        float v_dec = V_MIN + (speed - V_MIN) * (rest / DECEL_DIST_DEG);
        int v = (int)min(min(v_acc, v_dec), (float)speed);
        v = max(v, V_MIN);

        // --- PID on external error ---
        float e = error_src ? error_src() : 0.0f;
        int u = lround(pid_reg(pid_ext, e, kp, ki, kd));
        u = constrain(u, -v / 2, v / 2);

        move(dir * (v + u), dir * (v - u));

        if (progress && millis() - t_cb >= 100)
        {
            t_cb = millis();
            progress(pa, pb, target);
        }

        delay(2);
    }

    stop();
    if (progress)
        progress(pa, pb, target);
    return ok;
}

// Turn in place by angle degrees (angle > 0 - clockwise, < 0 - counterclockwise).
// Speed profile and PID alignment - same as in move_deg.
bool turn_deg_tank(float angle, int speed, unsigned long timeout_ms, MoveProgressCb progress)
{
    float a0, b0;
    if (!readAngles(a0, b0))
        return false;

    pid_reset(pid_straight);

    int dir = (angle >= 0) ? 1 : -1;
    float target = fabsf(angle) * (TRACK_MM / WHEEL_D_MM);

    unsigned long t0 = millis();
    unsigned long t_cb = 0;

    float pa = 0, pb = 0;
    bool ok = false;

    while (millis() - t0 < timeout_ms)
    {
        float a, b;
        if (!readAngles(a, b))
        {
            stop();
            return false;
        }

        pa = dir * (a - a0);
        pb = -dir * (b - b0);
        float traveled = 0.5f * (pa + pb);

        if (traveled >= target)
        {
            ok = true;
            break;
        }

        float rest = target - traveled;
        float v_acc = V_MIN + (speed - V_MIN) * (traveled / ACCEL_DIST_DEG);
        float v_dec = V_MIN + (speed - V_MIN) * (rest / DECEL_DIST_DEG);
        int v = (int)min(min(v_acc, v_dec), (float)speed);
        v = max(v, V_MIN);

        // --- PID wheel alignment ---
        float e = pa - pb;
        int u = lround(pid_reg(pid_straight, e, straight_p, straight_i, straight_d));
        u = constrain(u, -v / 2, v / 2);

        move(dir * (v - u), -dir * (v + u));

        if (progress && millis() - t_cb >= 100)
        {
            t_cb = millis();
            progress(pa, pb, target);
        }

        delay(2);
    }

    stop();
    if (progress)
        progress(pa, pb, target);
    return ok;
}

// Turn around one wheel (pivot turn).
// angle > 0: the body turns right (clockwise), angle < 0 - left.
// dir = true:  driving forward  (right -> left wheel A drives)
// dir = false: driving backward (right -> right wheel B drives, in reverse)
// The stationary wheel holds its position (speed 0).
bool turn_deg_pivot(float angle, int speed, bool dir,
                    unsigned long timeout_ms, MoveProgressCb progress)
{
    float a0, b0;
    if (!readAngles(a0, b0))
        return false;

    bool right = (angle >= 0);
    bool driveA = dir ? right : !right;

    int wdir = dir ? 1 : -1;

    float target = fabsf(angle) * (1.95f * TRACK_MM / WHEEL_D_MM);

    unsigned long t0 = millis();
    unsigned long t_cb = 0;

    float pa = 0, pb = 0;
    bool ok = false;

    while (millis() - t0 < timeout_ms)
    {
        float a, b;
        if (!readAngles(a, b))
        {
            stop();
            return false;
        }

        pa = wdir * (a - a0);
        pb = wdir * (b - b0);
        float traveled = driveA ? pa : pb;

        if (traveled >= target)
        {
            ok = true;
            break;
        }

        float rest = target - traveled;
        float v_acc = 5 + (speed - 5) * (traveled / 2000);
        float v_dec = 5 + (speed - 5) * (rest / 2000);
        int v = (int)min(min(v_acc, v_dec), (float)speed);
        v = max(v, 5);

        if (driveA)
            move(wdir * v, 0);
        else
            move(0, wdir * v);

        if (progress && millis() - t_cb >= 100)
        {
            t_cb = millis();
            progress(pa, pb, target);
        }

        delay(2);
    }

    stop();
    if (progress)
        progress(pa, pb, target);
    return ok;
}