#include <Arduino.h>
#include <Sharing.h>

void share_init()
{
    Serial.begin(115200);
}

static char rxLine[128];
static size_t rxPos = 0;

bool share_read_line(char *out, size_t out_size)
{
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            rxLine[rxPos] = '\0';

            if (rxPos > 0)
            {
                strncpy(out, rxLine, out_size - 1);
                out[out_size - 1] = '\0';
                rxPos = 0;
                return true;
            }

            rxPos = 0;
            continue;
        }

        if (rxPos < sizeof(rxLine) - 1)
        {
            rxLine[rxPos++] = c;
        }
        // on overflow — just ignore the extra character,
        // BUT DO NOT reset rxPos. That way only the current line gets corrupted,
        // not the next one.
    }

    return false;
}