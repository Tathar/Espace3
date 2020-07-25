#ifndef RESISTANCE_H
#define RESISTANCE_H

#include "config.h"
#include <CLI.h>

class Resistance : CLI_Command
{
public:
    Resistance(CLI &cli) : CLI_Command(cli,
                                       PSTR("res"),
                                       PSTR("resistance"),
                                       PSTR("Usage:\tresistance <commande>\n"
                                            "Where:\t<commande>\t 0, 1, 2, 3, 23, 4, get")){};

    void setup()
    {
        pinMode(PIN_RC_I_1, INPUT);      // set pin to input /!\ logique inverse
        pinMode(PIN_RC_I_23, INPUT);     // set pin to input /!\ logique inverse
        pinMode(PIN_RC_I_4, INPUT);      // set pin to input /!\ logique inverse
        digitalWrite(PIN_RC_I_1, HIGH);  //active pullup
        digitalWrite(PIN_RC_I_23, HIGH); //active pullup
        digitalWrite(PIN_RC_I_4, HIGH);  //active pullup

        pinMode(PIN_RC_O_1, OUTPUT);     // set pin to output
        pinMode(PIN_RC_O_23, OUTPUT);    // set pin to output
        pinMode(PIN_RC_O_4, OUTPUT);     // set pin to output
        digitalWrite(PIN_RC_O_1, HIGH);  //logique inverse
        digitalWrite(PIN_RC_O_23, HIGH); //logique inverse
        digitalWrite(PIN_RC_O_4, HIGH);  //logique inverse
    }

    int get()
    {
        int ret = digitalRead(PIN_RC_I_1) == LOW ? RC_1 : 0;
        ret += digitalRead(PIN_RC_I_23) == LOW ? RC_23 : 0;
        ret += digitalRead(PIN_RC_I_4) == LOW ? RC_4 : 0;
        return ret;
    }

    //defini quels resistance doive etre allumé
    //
    void set(int mode)
    {
        if ((mode & RC_1) == RC_1)
        {
            digitalWrite(PIN_RC_O_1, LOW);
        }
        else
        {
            digitalWrite(PIN_RC_O_1, HIGH);
        }

        if ((mode & RC_23) == RC_23)
        {
            digitalWrite(PIN_RC_O_23, LOW);
        }
        else
        {
            digitalWrite(PIN_RC_O_23, HIGH);
        }

        if ((mode & RC_4) == RC_4)
        {
            digitalWrite(PIN_RC_O_4, LOW);
        }
        else
        {
            digitalWrite(PIN_RC_O_4, HIGH);
        }
    }

    //  // CLI set parametre
    bool setparams(const char *params)
    {
        _params = String(params).toInt();
        return (params);
    }
    // CLI Execute
    bool execute(CLI &cli)
    {
        if (_params == 0)
        {
            Serial.println(F("resistance = 0"));
            set(RC_OFF);
        }
        else if (_params == 1)
        {
            Serial.println(F("resistance = 1"));
            set(RC_1);
        }
        else if (_params == 2)
        {
            Serial.println(F("resistance = 23"));
            set(RC_23);
        }
        else if (_params == 3)
        {
            Serial.println(F("resistance = 23"));
            set(RC_23);
        }
        else if (_params == 23)
        {
            Serial.println(F("resistance = 23"));
            set(RC_23);
        }
        else if (_params == 4)
        {
            Serial.println(F("resistance = 4"));
            set(RC_4);
        }

        // cli.print_P(PSTR("Autoradio "));
        // cli.println(_params);
        return false;
    }

private:
    int _params;
};

#endif