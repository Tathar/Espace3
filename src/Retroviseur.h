#ifndef RETROVISEUR_H
#define RETROVISEUR_H

#include "config.h"
#include <CLI.h>

class Retroviseur : CLI_Command
{
public:
    Retroviseur(CLI &cli) : CLI_Command(cli,
                                        PSTR("retro"),
                                        PSTR("retroviseur"),
                                        PSTR("Usage:\tretro <commande>\n"
                                             "Where:\t<commande>\t open, close"))
    {
        _open = false;
    };

    void setup()
    {
        pinMode(PIN_RV_CLOSE, OUTPUT); // set pin to output
        pinMode(PIN_RV_OPEN, OUTPUT);  // set pin to output
        digitalWrite(PIN_RV_CLOSE, LOW);
        digitalWrite(PIN_RV_OPEN, LOW);
        // open(); //todo
    }

    void loop()
    {
        if (this->_timer.front())
            stop();
    }

    void open()
    {
        if (!_open)
        {
            digitalWrite(PIN_RV_CLOSE, LOW);
            digitalWrite(PIN_RV_OPEN, HIGH);
            _timer.reset();
            _timer.start(RV_TIME);
            _open = true;
        }
    }

    void close()
    {
        if (_open)
        {
            digitalWrite(PIN_RV_OPEN, LOW);
            digitalWrite(PIN_RV_CLOSE, HIGH);
            _timer.reset();
            _timer.start(RV_TIME);
            _open = false;
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