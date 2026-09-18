#include <Arduino.h>

// -------------------- EC11 на модуле (синяя плата, KY-040-стиль) --------------------
// Отличия модуля от "голого" EC11:
//  - на плате стоят подтяжки 10k к VCC на CLK/DT (иногда и RC-фильтр),
//    фронты затянуты - старый метод "проверить пин через 300us" пропускал шаги;
//  - у модуля один полный квадратурный цикл (4 перехода) на один щелчок,
//    у голого EC11 из старой схемы выходило 2 перехода на щелчок -
//    отсюда "два щелчка = одно событие".
// Решение: полный табличный квадратурный декодер (x4) + делитель шагов на щелчок.

const int ENC_A = 4;
const int ENC_B = 2;
const int ENC_SW = 3;

// Сколько квадратурных переходов приходится на один щелчок (детент).
// Модуль KY-040/EC11: 4. Если события будут идти через щелчок - поставь 2.
const int STEPS_PER_DETENT = 4;

enum EncEvent
{
    ENC_NONE = 0,

    ENC_RIGHT = 1,
    ENC_LEFT = 2,

    ENC_RIGHT_HOLD = 3,
    ENC_LEFT_HOLD = 4,

    ENC_BUTTON_PRESSED = 5,
    ENC_BUTTON_RELEASED = 6
};

// -------------------- состояние --------------------
static uint8_t prevState = 0;   // предыдущее состояние (A<<1)|B
static int8_t stepAcc = 0;      // аккумулятор квадратурных шагов

static bool btnStable = HIGH;   // подтверждённое состояние кнопки
static bool btnLastRaw = HIGH;  // последнее сырое чтение
static unsigned long btnLastChange = 0;
const unsigned long BTN_DEBOUNCE_MS = 20;

// Таблица переходов квадратуры: индекс = (prev<<2)|curr,
// значение = +1 (по часовой), -1 (против), 0 (нет движения/дребезг).
static const int8_t QUAD_TABLE[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};

static inline uint8_t readAB()
{
    return (uint8_t)((digitalRead(ENC_A) << 1) | digitalRead(ENC_B));
}

void encoder_init()
{
    // Подтяжки оставляем включёнными: на CLK/DT они продублируют платные
    // (не мешает), а на SW у многих модулей своей подтяжки нет - там она обязательна.
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);

    prevState = readAB();
    stepAcc = 0;

    btnStable = digitalRead(ENC_SW);
    btnLastRaw = btnStable;
    btnLastChange = millis();
}

EncEvent encoder_read()
{
    // ---------- кнопка: неблокирующий антидребезг ----------
    bool raw = digitalRead(ENC_SW); // LOW = нажата

    if (raw != btnLastRaw)
    {
        btnLastRaw = raw;
        btnLastChange = millis();
    }
    else if (raw != btnStable && (millis() - btnLastChange) >= BTN_DEBOUNCE_MS)
    {
        btnStable = raw;
        return (btnStable == LOW) ? ENC_BUTTON_PRESSED : ENC_BUTTON_RELEASED;
    }

    // ---------- поворот: табличный квадратурный декодер ----------
    uint8_t curr = readAB();

    if (curr != prevState)
    {
        int8_t dir = QUAD_TABLE[(prevState << 2) | curr];
        prevState = curr;

        if (dir != 0)
        {
            stepAcc += dir;

            // Событие отдаём, только когда набрался полный щелчок
            // И энкодер стоит в детенте (оба канала HIGH у EC11 в покое) -
            // это отсекает половинчатые шаги при медленном вращении.
            if (stepAcc >= STEPS_PER_DETENT)
            {
                stepAcc = 0;
                bool hold = (btnStable == LOW);
                return hold ? ENC_RIGHT_HOLD : ENC_RIGHT;
            }
            if (stepAcc <= -STEPS_PER_DETENT)
            {
                stepAcc = 0;
                bool hold = (btnStable == LOW);
                return hold ? ENC_LEFT_HOLD : ENC_LEFT;
            }
        }
        else
        {
            // Недопустимый переход (пропуск/дребезг) - сбрасываем накопление,
            // чтобы не насчитать фантомный шаг.
            stepAcc = 0;
        }
    }

    return ENC_NONE;
}