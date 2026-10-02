#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

#include <Sharing.h>
#include <Interaction.h>
#include <Kicker.h>
#include <MotorDriver.h>

// ==================== Teleoperation from Pi ====================
// Protocol (lines from Pi over USB Serial):
//   "MOV,<a>,<b>\n" - motor speeds; sent continuously as a keepalive
//   "KICK\n"        - single kick
// Safety: if MOV doesn't arrive for longer than DEADMAN_MS - stop.

static const unsigned long DEADMAN_MS = 400;

static int cur_a = 0, cur_b = 0;
static unsigned long last_mov_ms = 0;
static bool link_alive = false;

static void process_line(char *line)
{
    char *cmd = strtok(line, ",");
    if (cmd == nullptr)
        return;

    if (strcmp(cmd, "MOV") == 0)
    {
        char *aStr = strtok(nullptr, ",");
        char *bStr = strtok(nullptr, ",");
        if (!aStr || !bStr)
            return;

        cur_a = atoi(aStr);
        cur_b = atoi(bStr);
        last_mov_ms = millis();

        move(cur_a, cur_b);
    }
    else if (strcmp(cmd, "KICK") == 0)
    {
        stop();
        kick();
    }
}

static void draw_status()
{
    oled_clear();
    oled_text(0, 0, "claude neuro", 1);
    if (link_alive)
    {
        oled_text(0, 16, "A: " + String(cur_a), 2);
        oled_text(0, 40, "B: " + String(cur_b), 2);
    }
    else
    {
        oled_text(0, 24, "no link", 2);
    }
    oled_show();
}

void setup()
{
    Serial.begin(115200); // first line - USB CDC
    motor_init();
    kick_init();
    interaction_init();

    oled_clear();
    oled_text(0, 0, "claude", 1);
    oled_text(0, 24, "waiting...", 1);
    oled_show();
}

void loop()
{
    char line[32];
    while (share_read_line(line, sizeof(line)))
    {
        process_line(line);
    }

    // deadman: link lost - stop
    bool alive_now = (millis() - last_mov_ms) < DEADMAN_MS;
    if (!alive_now && link_alive)
    {
        stop();
        cur_a = 0;
        cur_b = 0;
    }

    // redraw the screen no more than 5 times/sec and only on changes
    static unsigned long t_draw = 0;
    static int drawn_a = 999, drawn_b = 999;
    static bool drawn_alive = false;

    if (millis() - t_draw >= 200 &&
        (drawn_a != cur_a || drawn_b != cur_b || drawn_alive != alive_now))
    {
        t_draw = millis();
        link_alive = alive_now;
        drawn_a = cur_a;
        drawn_b = cur_b;
        drawn_alive = alive_now;
        draw_status();
    }
    else
    {
        link_alive = alive_now;
    }

    delay(2);
}