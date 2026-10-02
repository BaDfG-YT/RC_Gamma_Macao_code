#pragma once

#include <Arduino.h>
#include <Encoder.h>

extern EncEvent g_menu_enc_event;

// callback function type for a menu item
typedef void (*MenuAction)();

// item type
enum MenuItemType
{
    ITEM_FOLDER,        // contains sub-items
    ITEM_ACTION_ONCE,   // execute and return
    ITEM_ACTION_LOOP    // run in a loop, exit by button
};

// In MenuItem — add new fields at the end of the struct (old initializer
// literals won't break since the field defaults to nullptr):
struct MenuItem
{
    const char *title;
    MenuItemType type;
    const MenuItem *children;
    int children_count;
    void (*action)();
    void (*on_enter)();
    void (*on_exit)();
    void *data = nullptr;   // extra data for generic items (ChoiceSetter* etc.)
};

// Choosing one value from a fixed set of options
struct ChoiceSetter
{
    const char *title;    // title shown on screen
    int *target;          // where the chosen value is stored
    const int *values;    // options
    int count;             // how many options
    int index;              // current index (internal state)
};

// Generic handlers for a choice-menu item.
// An item is created like this:
//   {"Turn", ITEM_ACTION_LOOP, nullptr, 0, choice_loop, choice_enter, choice_exit, &cs_myvar}
void choice_enter();
void choice_loop();
void choice_exit();
int choice_select(const char *title, const int *values, int count, int default_index = 0);

void menu_init(const MenuItem *root, int root_count);
void menu_loop();

// Blocking screen for choosing an integer with a given step and bounds.
// Turn the encoder - the number increases/decreases by step, press the button - confirm and exit.
// Returns the chosen value.
int number_select(const char *title, int v_min, int v_max, int step, int default_value);