#include <Arduino.h>
#include <MotorDriver.h>
#include <Interaction.h>

int speed = 35;

// ---------------- твоя функция движения ----------------
void move_angle(float ang)
{
    int pA = speed * sin(radians(ang - 62.5));
    int pB = speed * sin(radians(ang - 117.5));
    int pC = speed * sin(radians(ang + 62.5));
    int pD = speed * sin(radians(ang + 117.5));

    move(pA, pB, pC, pD);
}

// ---------------- протокол Pi5 -> Pico ----------------
float g_cmd_angle = 0.0f;
bool g_arrived = true;
unsigned long g_last_cmd_ms = 0;

void process_line(String line)
{
    line.trim();

    if (!line.startsWith("CMD,"))
        return;

    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);

    if (p1 < 0 || p2 < 0)
        return;

    String s_angle = line.substring(p1 + 1, p2);
    String s_arrived = line.substring(p2 + 1);

    g_cmd_angle = s_angle.toFloat();
    g_arrived = (s_arrived.toInt() != 0);
    g_last_cmd_ms = millis();

    Serial.print("RX angle=");
    Serial.print(g_cmd_angle);
    Serial.print(" arrived=");
    Serial.println(g_arrived ? 1 : 0);
}

void read_usb_commands()
{
    static String line = "";

    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (c == '\n')
        {
            process_line(line);
            line = "";
        }
        else if (c != '\r')
        {
            line += c;
        }
    }
}

void setup()
{
    strip_init();
    smile();
    Serial.begin(115200);
    delay(2000);

    motor_init();
    stop();

    g_last_cmd_ms = millis();
}

void loop()
{
    read_usb_commands();

    // watchdog: если Pi 5 перестала слать команды — остановиться
    if (millis() - g_last_cmd_ms > 500)
    {
        stop();
        return;
    }

    if (g_arrived)
    {
        stop();
        strip_fill(255, 0, 0);
    }
    else
    {
        move_angle(g_cmd_angle);
        int lind = val_to_ledind(g_cmd_angle, 360);
        led_set(lind, 0, 0, 255, 1);
    }

    strip_show();
    delay(10);
}