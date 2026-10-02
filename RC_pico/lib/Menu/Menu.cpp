#include "Menu.h"
#include <Interaction.h>
#include <Encoder.h>

// navigation stack — which folders are open
static const int MAX_DEPTH = 4;

struct NavLevel
{
    const MenuItem *items;
    int count;
    int cursor; // highlighted item
    int scroll; // topmost visible item
};

static NavLevel nav_stack[MAX_DEPTH];
static int nav_depth = 0;

// mode: browsing the menu or running a LOOP action
static enum {
    MODE_BROWSE,
    MODE_RUNNING_LOOP
} mode = MODE_BROWSE;

static const MenuItem *running_item = nullptr;

// how many items fit on a 128x64 OLED screen
// at font size 1 a line is ~10px, header 12px → 5 visible lines
static const int VISIBLE_ROWS = 5;
static const int ROW_HEIGHT = 10;

static NavLevel &current_level()
{
    return nav_stack[nav_depth - 1];
}

EncEvent g_menu_enc_event = ENC_NONE;

static void clamp_scroll(NavLevel &lvl)
{
    if (lvl.cursor < lvl.scroll)
        lvl.scroll = lvl.cursor;
    if (lvl.cursor >= lvl.scroll + VISIBLE_ROWS)
        lvl.scroll = lvl.cursor - VISIBLE_ROWS + 1;
    if (lvl.scroll < 0)
        lvl.scroll = 0;
}

static void draw_menu()
{
    oled_clear();

    NavLevel &lvl = current_level();

    // header: path
    // show the current folder's name, or "Main" for the root
    char header[24];
    if (nav_depth == 1)
    {
        snprintf(header, sizeof(header), "Main");
    }
    else
    {
        const char *parent_title = nav_stack[nav_depth - 2].items[nav_stack[nav_depth - 2].cursor].title;
        snprintf(header, sizeof(header), "< %s", parent_title);
    }
    oled_text(0, 0, header, 1);

    // separator
    // (a line could be drawn with Adafruit_GFX, but for simplicity we skip it)

    for (int row = 0; row < VISIBLE_ROWS; row++)
    {
        int idx = lvl.scroll + row;
        if (idx >= lvl.count)
            break;

        int y = 12 + row * ROW_HEIGHT;
        bool selected = (idx == lvl.cursor);

        // prefix: ">" for the selected item, " " for the rest
        char line[24];
        snprintf(line, sizeof(line), "%c%s", selected ? '>' : ' ', lvl.items[idx].title);

        oled_text(0, y, line, 1);
    }

    oled_show();
}

static void draw_running(const MenuItem *it)
{
    oled_clear();
    oled_text(0, 0, "Running:", 1);
    oled_text(0, 16, it->title, 2);
    oled_text(0, 50, "Press to exit", 1);
    oled_show();
}

void menu_init(const MenuItem *root, int root_count)
{
    encoder_init();

    nav_depth = 1;
    nav_stack[0].items = root;
    nav_stack[0].count = root_count;
    nav_stack[0].cursor = 0;
    nav_stack[0].scroll = 0;

    mode = MODE_BROWSE;
    running_item = nullptr;

    draw_menu();
}

static void enter_item()
{
    NavLevel &lvl = current_level();
    const MenuItem *it = &lvl.items[lvl.cursor];

    switch (it->type)
    {
    case ITEM_FOLDER:
        if (nav_depth < MAX_DEPTH && it->children_count > 0)
        {
            nav_stack[nav_depth].items = it->children;
            nav_stack[nav_depth].count = it->children_count;
            nav_stack[nav_depth].cursor = 0;
            nav_stack[nav_depth].scroll = 0;
            nav_depth++;
            draw_menu();
        }
        break;

    case ITEM_ACTION_ONCE:
        if (it->action)
            it->action();
        // stay at the same place in the menu
        draw_menu();
        break;

    case ITEM_ACTION_LOOP:
        running_item = it;
        mode = MODE_RUNNING_LOOP;
        if (it->on_enter)
            it->on_enter();
        draw_running(it);
        break;
    }
}

static void exit_loop_action()
{
    if (running_item && running_item->on_exit)
        running_item->on_exit();

    running_item = nullptr;
    mode = MODE_BROWSE;
    draw_menu();
}

static void go_back()
{
    if (nav_depth > 1)
    {
        nav_depth--;
        draw_menu();
    }
}

void menu_loop()
{
    EncEvent ev = encoder_read();

    if (mode == MODE_RUNNING_LOOP)
    {
        // forward the encoder event into action via the global
        g_menu_enc_event = ev;
        led_blink(1, 255);
        led_show();

        if (running_item && running_item->action)
            running_item->action();

        g_menu_enc_event = ENC_NONE;

        if (ev == ENC_BUTTON_RELEASED)
        {
            exit_loop_action();
        }
        return;
    }

    // mode == MODE_BROWSE
    NavLevel &lvl = current_level();

    switch (ev)
    {
    case ENC_RIGHT:
        if (lvl.cursor < lvl.count - 1)
        {
            lvl.cursor++;
            clamp_scroll(lvl);
            draw_menu();
        }
        // on the last item — do nothing (could wrap around, see below)
        break;

    case ENC_LEFT:
        if (lvl.cursor > 0)
        {
            lvl.cursor--;
            clamp_scroll(lvl);
            draw_menu();
        }
        else
        {
            // on the first item → go up one level
            go_back();
        }
        break;

    case ENC_LEFT_HOLD:
    case ENC_RIGHT_HOLD:
        // keep button-hold as an additional way to exit
        go_back();
        break;
    case ENC_BUTTON_RELEASED:
        enter_item();
        break;

    default:
        break;
    }
}

static void choice_draw(ChoiceSetter *cs)
{
    oled_clear();
    oled_text(0, 0, cs->title, 1);
    oled_text(0, 20, String(cs->values[cs->index]), 2);
    oled_text(0, 50, String(cs->index + 1) + "/" + String(cs->count), 1);
    oled_show();
}

void choice_enter()
{
    ChoiceSetter *cs = (ChoiceSetter *)running_item->data;
    if (!cs) return;

    // sync the cursor position with the variable's current value
    for (int i = 0; i < cs->count; i++)
        if (cs->values[i] == *cs->target)
        {
            cs->index = i;
            break;
        }

    choice_draw(cs);
}

void choice_loop()
{
    ChoiceSetter *cs = (ChoiceSetter *)running_item->data;
    if (!cs) return;

    bool changed = false;

    if (g_menu_enc_event == ENC_RIGHT)
    {
        cs->index = (cs->index + 1) % cs->count;
        changed = true;
    }
    else if (g_menu_enc_event == ENC_LEFT)
    {
        cs->index = (cs->index - 1 + cs->count) % cs->count;
        changed = true;
    }

    if (changed)
    {
        *cs->target = cs->values[cs->index];   // save immediately, live
        choice_draw(cs);
    }

    delay(10);
}


void choice_exit()
{
    // the value is already stored in *target - nothing to do here
}

int choice_select(const char *title, const int *values, int count, int default_index)
{
    int index = default_index;
    if (index < 0) index = 0;
    if (index >= count) index = count - 1;

    auto draw = [&]() {
        oled_clear();
        oled_text(0, 0, title, 1);
        oled_text(0, 20, String(values[index]), 2);
        oled_text(0, 50, String(index + 1) + "/" + String(count), 1);
        oled_show();
    };

    draw();

    while (true)
    {
        EncEvent ev = encoder_read();

        if (ev == ENC_RIGHT)
        {
            index = (index + 1) % count;
            draw();
        }
        else if (ev == ENC_LEFT)
        {
            index = (index - 1 + count) % count;
            draw();
        }
        else if (ev == ENC_BUTTON_RELEASED)
        {
            break;   // choice confirmed
        }

        delay(10);
    }

    return values[index];
}

int number_select(const char *title, int v_min, int v_max, int step, int default_value)
{
    int value = constrain(default_value, v_min, v_max);

    auto draw = [&]() {
        oled_clear();
        oled_text(0, 0, title, 1);
        oled_text(0, 20, String(value), 2);
        String range_str = String(v_min) + ".." + String(v_max);
        oled_text(0, 50, range_str, 1);
        oled_show();
    };

    draw();

    while (true)
    {
        EncEvent ev = encoder_read();

        if (ev == ENC_RIGHT)
        {
            value += step;
            if (value > v_max) value = v_max;
            draw();
        }
        else if (ev == ENC_LEFT)
        {
            value -= step;
            if (value < v_min) value = v_min;
            draw();
        }
        else if (ev == ENC_BUTTON_RELEASED)
        {
            break; // choice confirmed
        }

        delay(10);
    }

    return value;
}