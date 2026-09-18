#pragma once

#include <Arduino.h>
#include <Encoder.h>

extern EncEvent g_menu_enc_event;

// тип callback-функции для пункта меню
typedef void (*MenuAction)();

// тип пункта
enum MenuItemType
{
    ITEM_FOLDER,        // содержит подпункты
    ITEM_ACTION_ONCE,   // выполнить и вернуться
    ITEM_ACTION_LOOP    // выполнять в цикле, выход по кнопке
};

// В MenuItem — новое поле в конец структуры (старые литералы не сломаются,
// у поля дефолт nullptr):
struct MenuItem
{
    const char *title;
    MenuItemType type;
    const MenuItem *children;
    int children_count;
    void (*action)();
    void (*on_enter)();
    void (*on_exit)();
    void *data = nullptr;   // доп. данные для generic-пунктов (ChoiceSetter* и т.п.)
};

// Выбор одного значения из фиксированного набора вариантов
struct ChoiceSetter
{
    const char *title;    // заголовок на экране
    int *target;          // куда сохраняется выбранное значение
    const int *values;    // варианты
    int count;             // сколько вариантов
    int index;              // текущий индекс (внутреннее состояние)
};

// Generic-обработчики для пункта меню-выбора.
// Пункт создаётся так:
//   {"Turn", ITEM_ACTION_LOOP, nullptr, 0, choice_loop, choice_enter, choice_exit, &cs_myvar}
void choice_enter();
void choice_loop();
void choice_exit();
int choice_select(const char *title, const int *values, int count, int default_index = 0);

void menu_init(const MenuItem *root, int root_count);
void menu_loop();

// Блокирующий экран выбора целого числа с заданным шагом и границами.
// Крутишь энкодер - число растёт/падает на step, нажал кнопку - подтвердил и вышел.
// Возвращает выбранное значение.
int number_select(const char *title, int v_min, int v_max, int step, int default_value);