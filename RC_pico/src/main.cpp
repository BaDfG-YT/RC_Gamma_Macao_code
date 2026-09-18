#include <Arduino.h>
#include <Menu.h>
#include <Interaction.h>
#include <Kicker.h>
#include <Encoder.h>
#include <MotorDriver.h>
#include <Positioning.h>
#include <Regulators.h>

// источник ошибки: отклонение красного (свежее чтение каждый вызов)
float red_error()
{
    pos_read_cam();
    return g_red_visible ? g_red_dx : 0.0f; // нет цели - едем прямо
}

// === ACTION-функции ===

// одноразовые
void act_kick()
{
    delay(1000);
    kick();
}

// Отрисовка прогресса поездки (колбэк для move_deg)
void draw_move_progress(float pa, float pb, float target)
{
    oled_clear();
    oled_text(0, 0, "Moving...", 1);
    oled_text(0, 16, "A: " + String(pa, 0), 1);
    oled_text(0, 28, "B: " + String(pb, 0), 1);
    oled_text(0, 44, String(0.5f * (pa + pb), 0) + " / " + String(target, 0), 1);
    oled_show();
}

void move_fwd()
{
    bool ok;
    ok = move_mm(500, 25, 15000, draw_move_progress);

    ok = turn_deg_tank(360, 20, 25000, draw_move_progress);

    oled_clear();
    oled_text(0, 0, ok ? "Move done" : "Move FAIL", 1);
    oled_show();
    delay(700);
}

// луповые

void loop_show_enc()
{
    float a, b;
    bool ok = readAngles(a, b); // накопленные градусы, без переполнений

    oled_clear();
    oled_text(0, 0, "Encoders", 1);
    if (ok)
    {
        oled_text(0, 16, "A: " + String(a, 1), 1); // градусы
        oled_text(0, 28, "B: " + String(b, 1), 1);
        oled_text(0, 44, "tA: " + String(a / 360.0f, 2) + " tB: " + String(b / 360.0f, 2), 1); // обороты
    }
    else
    {
        oled_text(0, 24, "CAN timeout!", 1);
    }
    oled_show();
    delay(50);
}

// ITEM_ACTION_LOOP: показывает отклонение красного от центра по ширине
void loop_show_red()
{
    pos_read_cam();

    oled_clear();
    if (g_red_visible)
    {
        oled_text(0, 0, "RED target", 1);
        oled_text(0, 16, "dx: " + String(g_red_dx, 1), 2);
        oled_text(0, 40, "area: " + String(g_red_area), 1);
    }
    else
    {
        oled_text(0, 0, "RED target", 1);
        oled_text(0, 24, "no red", 2);
    }
    oled_show();
}

PID pid_bowls;
float bowl_p = 0.068f, bowl_ki = 0.0007f, bowl_d = 0.7f; // ki с нуля, подберёшь на роботе
float bowl_p_dyn = 0.025f, bowl_i_dyn = 0.000f, bowl_d_dyn = 0.5f;

void ent_obs_bowls()
{
    pid_reset(pid_bowls);
}

void obs_bowls()
{
    pos_read_cam();

    oled_clear();
    if (g_red_visible)
    {
        obsession(constrain(pid_reg(pid_bowls, g_red_dx, bowl_p, bowl_ki, bowl_d), -1, 1));
        oled_text(0, 0, "RED target", 1);
        oled_text(0, 16, "dx: " + String(g_red_dx, 1), 2);
        oled_text(0, 40, "area: " + String(g_red_area), 1);
    }
    else
    {
        stop();
        oled_text(0, 0, "RED target", 1);
        oled_text(0, 24, "no red", 2);
    }
    oled_show();
}

void shot_bowls()
{
    delay(600);
    pos_read_cam();

    static constexpr float DX_TOL = 10.0f;                  // допуск по центру, px
    static constexpr unsigned long HOLD_MS = 800;           // сколько держать центр перед ударом
    static constexpr unsigned long SHOT_TIMEOUT_MS = 10000; // предохранитель

    unsigned long t_start = millis();
    unsigned long t_in_center = 0; // когда вошли в допуск (0 = не в допуске)
    bool ready = false;

    while (!ready)
    {
        if (millis() - t_start > SHOT_TIMEOUT_MS)
        {
            // не удалось стабильно навестись - выходим без удара
            stop();
            kick();
            oled_clear();
            oled_text(0, 0, "Shot TIMEOUT", 1);
            oled_show();
            delay(500);
            return;
        }

        pos_read_cam();

        oled_clear();
        if (g_red_visible)
        {
            obsession(constrain(pid_reg(pid_bowls, g_red_dx, bowl_p, bowl_ki, bowl_d), -2, 2));

            bool in_center = (fabsf(g_red_dx) <= DX_TOL);

            if (in_center)
            {
                if (t_in_center == 0)
                    t_in_center = millis(); // только что вошли в допуск

                unsigned long held = millis() - t_in_center;
                if (held >= HOLD_MS)
                    ready = true;

                oled_text(0, 0, "RED - CENTERED", 1);
                oled_text(0, 16, "dx: " + String(g_red_dx, 1), 1);
                oled_text(0, 30, "hold: " + String(held) + "/" + String(HOLD_MS), 1);
            }
            else
            {
                t_in_center = 0; // вышли из допуска - таймер сбрасывается
                oled_text(0, 0, "RED target", 1);
                oled_text(0, 16, "dx: " + String(g_red_dx, 1), 2);
                oled_text(0, 40, "area: " + String(g_red_area), 1);
            }
        }
        else
        {
            stop();
            t_in_center = 0; // цель потеряна - таймер сбрасывается
            oled_text(0, 0, "RED target", 1);
            oled_text(0, 24, "no red", 2);
        }
        oled_show();
    }

    stop();
    delay(500);
    kick();
}

// игра

static const int ZONE_OPTIONS[] = {-1, 1}; // -1 - левая зона старта, 1 - правая
static const int TURN_OPTIONS[] = {0, 90, -90, 180};

int speed = 100;
const int deg_t1 = 120, deg_t2 = 120;
void game()
{
    // int start_zone = choice_select("Start Zone", ZONE_OPTIONS, 2);
    // int turn_deg_choice = choice_select("Turn deg", TURN_OPTIONS, 4);

    int start_zone = -1;
    int turn_deg_choice = -90;

    int dist1, dist2;
    int dist1_offset = number_select("Dist1 adj", -100, 100, 5, 0);
    int dist2_offset = number_select("Dist2 adj", -100, 100, 5, 0);

    delay(50);

    // if (start_zone == 1)
    // {
    //     turn_deg_tank(turn_deg_choice == -90 ? 0 : turn_deg_choice - 90, speed);

    //     if (turn_deg_choice == 0)
    //     {
    //         dist1 = 160;
    //         dist2 = 320;
    //     }
    //     else if (turn_deg_choice == 180)
    //     {
    //         dist1 = 160;
    //         dist2 = 330;
    //     }
    //     else if (turn_deg_choice == 90)
    //     {
    //         dist1 = -163 - 26;
    //         dist2 = 320;
    //     }
    //     else if (turn_deg_choice == -90)
    //     {
    //         dist1 = 160;
    //         dist2 = 320;
    //     }
    //     dist1 += dist1_offset;
    //     move_mm(dist1, speed);
    //     turn_deg_tank(turn_deg_choice == 90 ? -90 : 90, speed);
    // }
    // else
    // {
    //     turn_deg_tank(turn_deg_choice == -90 ? 0 : 90 - turn_deg_choice, speed);

    //     if (turn_deg_choice == 0)
    //     {
    //         dist1 = 160;
    //         dist2 = 320;
    //     }
    //     else if (turn_deg_choice == 180)
    //     {
    //         dist1 = 160;
    //         dist2 = 330;
    //     }
    //     else if (turn_deg_choice == 90)
    //     {
    //         dist1 = 163;
    //         dist2 = 320;
    //     }
    //     else if (turn_deg_choice == -90)
    //     {
    //         dist1 = -163 - 26;
    //         dist2 = 320;
    //     }

    //     dist1 += dist1_offset;
    //     move_mm(dist1, speed);
    //     turn_deg_tank(turn_deg_choice == -90 ? 90 : -90, speed);
    // }

    dist1 = -173;
    dist2 = 320;
    // turn_deg_tank(180, speed);
    move_mm(dist1 + dist1_offset, 20);
    turn_deg_tank(90, speed);

    dist2 += dist2_offset;
    move_mm_pid(dist2, red_error, bowl_p_dyn, bowl_i_dyn, bowl_d_dyn, 150, 8000, draw_move_progress);

    shot_bowls();

    move_mm(-220, 200);
}

void game2()
{
    move_mm_pid(220, red_error, bowl_p_dyn, bowl_i_dyn, bowl_d_dyn, 150, 8000, draw_move_progress);
    shot_bowls();
    move_mm(-220, 200);
}

// === Декларация структуры меню ===

void act_stop()
{
    stop();
}

void tst_func()
{
    turn_deg_pivot(360 * 1, 50, 1, 25000);
    delay(1000);
    turn_deg_pivot(-360 * 1, 50, 1, 25000);
}

// func_test → подпункты
const MenuItem func_test_items[] = {
    {"Test func", ITEM_ACTION_ONCE, nullptr, 0, tst_func, nullptr, act_stop},
    {"Obs bowls", ITEM_ACTION_LOOP, nullptr, 0, obs_bowls, ent_obs_bowls, act_stop},
    {"Shot bowls", ITEM_ACTION_ONCE, nullptr, 0, shot_bowls, ent_obs_bowls, act_stop},
    {"Kick once", ITEM_ACTION_ONCE, nullptr, 0, act_kick, nullptr, nullptr},
    {"Move fwd", ITEM_ACTION_ONCE, nullptr, 0, move_fwd, nullptr, act_stop},
    {"Red dx", ITEM_ACTION_LOOP, nullptr, 0, loop_show_red, nullptr, nullptr},
    {"Enc Show", ITEM_ACTION_LOOP, nullptr, 0, loop_show_enc, nullptr, nullptr},
    {"Block motors", ITEM_ACTION_ONCE, nullptr, 0, act_stop, nullptr, act_stop},

    // {"Cam info", ITEM_ACTION_LOOP, nullptr, 0, loop_show_cam, nullptr, nullptr},
};

// game → подпункты
const MenuItem game_items[] = {
    {"GAME1", ITEM_ACTION_ONCE, nullptr, 0, game, ent_obs_bowls, act_stop},
    {"GAME2", ITEM_ACTION_ONCE, nullptr, 0, game2, ent_obs_bowls, act_stop},
};

struct NumSetter
{
    const char *title;
    float *target; // на что указываем
    float v_min;
    float v_max;
    float step;
};

static NumSetter *active_setter = nullptr;

static void draw_setter_ui()
{
    if (!active_setter || !active_setter->target)
        return;

    float v = *active_setter->target;

    oled_clear();
    oled_text(0, 0, active_setter->title, 1);
    oled_text(0, 16, "Value:", 1);
    oled_text(0, 32, String(v, 3), 2);

    // показать диапазон мелко
    String range_str = String(active_setter->v_min, 2) + ".." + String(active_setter->v_max, 2);
    oled_text(0, 54, range_str, 1);

    oled_show();
}

static void setter_enter()
{
    draw_setter_ui();
}

static void setter_loop()
{
    if (!active_setter || !active_setter->target)
        return;

    bool changed = false;

    if (g_menu_enc_event == ENC_RIGHT)
    {
        *active_setter->target += active_setter->step;
        if (*active_setter->target > active_setter->v_max)
            *active_setter->target = active_setter->v_max;
        changed = true;
    }
    else if (g_menu_enc_event == ENC_LEFT)
    {
        *active_setter->target -= active_setter->step;
        if (*active_setter->target < active_setter->v_min)
            *active_setter->target = active_setter->v_min;
        changed = true;
    }

    if (changed)
        draw_setter_ui();

    delay(10);
}

static NumSetter setter_bowl_p = {.title = "bowl p dyn", .target = &bowl_p_dyn, .v_min = 0.0f, .v_max = 5.0f, .step = 0.01f};
static NumSetter setter_bowl_i = {.title = "bowl i dyn", .target = &bowl_i_dyn, .v_min = 0.0f, .v_max = 0.01f, .step = 0.0001f};
static NumSetter setter_bowl_d = {.title = "bowl d dyn", .target = &bowl_d_dyn, .v_min = 0.0f, .v_max = 50.0f, .step = 0.1f};

static void enter_set_i()
{
    active_setter = &setter_bowl_i;
    setter_enter();
}

static NumSetter setter_bowl_d_dyn = {
    .title = "bowl d",
    .target = &bowl_d_dyn,
    .v_min = 0.0f,
    .v_max = 30.0f, // подгони под смысл переменной
    .step = 1.0f,
};

static void enter_set_p()
{
    active_setter = &setter_bowl_p;
    setter_enter();
}

static void enter_set_d()
{
    active_setter = &setter_bowl_d;
    setter_enter();
}

static void setter_exit()
{
    active_setter = nullptr;
}

const MenuItem coef_set_items[] = {
    // {"bowl p", ITEM_ACTION_LOOP, nullptr, 0, setter_loop, enter_set_p, setter_exit},
    // {"bowl i", ITEM_ACTION_LOOP, nullptr, 0, setter_loop, enter_set_i, setter_exit},
    // {"bowl d", ITEM_ACTION_LOOP, nullptr, 0, setter_loop, enter_set_d, setter_exit},
    {"bowl p DYN", ITEM_ACTION_LOOP, nullptr, 0, setter_loop, enter_set_p, setter_exit},
    {"bowl i DYN", ITEM_ACTION_LOOP, nullptr, 0, setter_loop, enter_set_i, setter_exit},
    {"bowl d DYN", ITEM_ACTION_LOOP, nullptr, 0, setter_loop, enter_set_d, setter_exit},
};

// корневое меню
const MenuItem root_menu[] = {
    {"func_test", ITEM_FOLDER, func_test_items, sizeof(func_test_items) / sizeof(MenuItem), nullptr, nullptr, nullptr},
    {"coef_set", ITEM_FOLDER, coef_set_items, sizeof(coef_set_items) / sizeof(MenuItem), nullptr, nullptr, nullptr},
    {"game", ITEM_FOLDER, game_items, sizeof(game_items) / sizeof(MenuItem), nullptr, nullptr, nullptr},
};

const int root_count = sizeof(root_menu) / sizeof(MenuItem);

void setup()
{
    Serial.begin(115200); // ПЕРВОЙ строкой
    motor_init();
    kick_init();
    interaction_init(); // OLED + LED strip
    oled_text(0, 0, "hello", 1);
    oled_show();
    menu_init(root_menu, root_count);

    led_set(255);
    led_show();
}

void loop()
{
    menu_loop();
}