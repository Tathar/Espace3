#ifndef RETROVISEUR_H
#define RETROVISEUR_H

#include "config.h"
#include <CLI.h>

#define MAP_F(x, in_min, in_max, out_min, out_max) (float((x - in_min) * (out_max - out_min)) / float(in_max - in_min) + out_min)

class Retroviseur : CLI_Command
{
public:
    Retroviseur(CLI &cli) : CLI_Command(cli,
                                        PSTR("retro"),
                                        PSTR("retroviseur"),
                                        PSTR("Usage:\tretro <commande>\n"
                                             "Where:\t<commande>\t open, close"))
    {
        _timer = NeoTimer(RV_TIME);
        _open = false;
    }

    void setup()
    {
        pinMode(PIN_RV_CLOSE, OUTPUT); // set pin to output
        digitalWrite(PIN_RV_OPEN, LOW);
        pinMode(PIN_RV_OPEN, OUTPUT); // set pin to output
        digitalWrite(PIN_RV_CLOSE, LOW);
    }

    void loop()
    {
        static int val_stop = MAP_F(RETRO_I_STOP, AMPEREMETRE_I_MIN, AMPEREMETRE_I_MAX, 0, 1023);
        // Serial.println(this->_timer.remaining());
        if (this->_timer.front())
        {
            Serial.println(F("retro stoped by Timer"));
            stop();
        }

        if (_timer.waiting() && millis() % 100 == 0)
        {
            Serial.print(F("I="));
            Serial.println((float)MAP_F(val, 0, 1023, -5, 5));
        }

        int val = analogRead(0);
        if (_timer.waiting() && ((val >= val_stop) || (val <= -val_stop)))
        {
            Serial.println(F("retro stoped by ampermetre "));
            Serial.println((float)MAP_F(val, 0, 1023, -5, 5));
            _timer.reset();
            stop();
        }
    }

    void open()
    {
        if (!_open)
        {
            digitalWrite(PIN_RV_CLOSE, LOW);
            digitalWrite(PIN_RV_OPEN, HIGH);
            _open = true;
            _timer.reset();
            _timer.start(RV_TIME);
        }
    }

    void close()
    {
        if (_open)
        {
            digitalWrite(PIN_RV_OPEN, LOW);
            digitalWrite(PIN_RV_CLOSE, HIGH);
            _open = false;
            _timer.reset();
            _timer.start(RV_TIME);
        }
    }

    void stop()
    {
        digitalWrite(PIN_RV_CLOSE, LOW);
        digitalWrite(PIN_RV_OPEN, LOW);
    }

    // // CLI set parametre
    bool setparams(const char *params)
    {
        _params = params;
        return (params);
    }
    // CLI Execute
    bool execute(CLI &cli)
    {
        if (strcmp(_params, "open") == 0)
        {
            Serial.println(F("action = open"));
            open();
        }
        else if (strcmp(_params, "close") == 0)
        {
            Serial.println(F("action = close"));
            close();
        }
        else if (strcmp(_params, "i") == 0)
        {
            int val = analogRead(0);
            Serial.println((float)MAP_F(val, 0, 1023, -5, 5));
        }
        else
        {
            Serial.println(F("action inconue"));
        }

        // cli.print_P(PSTR("Autoradio "));
        // cli.println(_params);
        return false;
    }

private:
    const char *_params;
    NeoTimer _timer;
    bool _open;
    // bool active;
    // unsigned long int l_time;
    // int l_wait;
    void *l_arg;
};

#endif