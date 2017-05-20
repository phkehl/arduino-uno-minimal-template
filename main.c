/*!
    \file
    \brief XXX short description goes here XXX

    Copyright (c) YEAR NAME NAME (CONTACT OR URL)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    \mainpage

    \section P_MAIN Add Title Here

    Add description here.

*/

#include "ffstuff.h"

int main(void)
{
    // initialise flipflip's Arduino Uno stuff
    ffInit();

    // say hello
    NOTICE("Super Demo Program");
    NOTICE("Copyright (c) YEAR NAME NAME (CONTACT OR URL)");

    // the Arduino UNO LED pin
    PIN_OUTPUT(_D13);

    // run our thing endlessly..
    while (ENDLESS)
    {
        {
            uint8_t n = 20;
            while (n--)
            {
                DEBUG("n=%"PRIu8, n);
                PIN_TOGGLE(_D13);
                _delay_ms(100);
            }
        }


        {
            _delay_ms(1000);
            ERROR("an error...");
            _delay_ms(1000);
            WARNING("a warning...");
            _delay_ms(1000);
        }

        {
            char str[50];
            ffStatus(str, sizeof(str));
            PRINT("ffStatus: %s", str);
        }

        {
            ffTic(0);
            uint16_t n = 23456;
            while (n--)
            {
            }
            const uint16_t dt = ffToc(0);
            PRINT("dt=%"PRIu16"us", dt);
        }

        {
            const char *foo = PSTR("romz stringz");
            const char *bar = "ram string";
            const int16_t i = -44;
            const float f = 1.234f;
            const uint16_t x = 0xabcd;
            PRINT("foo=%S bar=%s i=%"PRIi16" f=%g x=0x%04"PRIx16,
                foo, bar, i, f, x);
        }


    }

    // they say that embedded programs shall never return from main
    while (ENDLESS)
    {
    }
    return 0;
}


/* ************************************************************************** */
// eof
