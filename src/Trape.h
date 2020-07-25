#ifndef TRAPE_H
#define TRAPE_H

#include "config.h"
#include <CLI.h>

class Trape : CLI_Command
{
public:
    Trape(CLI &cli) : CLI_Command(cli,
                                  PSTR("trape"),
                                  PSTR("trape"),
                                  PSTR("Usage:\ttrape <commande>\n"
                                       "Where:\t<commande>\t open, close")){};

    void setup()
    {
        pinMode(PIN_TRAP_ALIM, OUTPUT); // set pin to output
        digitalWrite(PIN_TRAP_ALIM, LOW);

        this->_servo.attach(PIN_TRAP_PWM);
        this->_servo.write(TRAP_STOP);
        Serial.println(this->_servo.read());
        active = false;
        this->_timer.reset();
        this->_open = false;
    }

    void loop()
    {
        //   if (active)
        //   {
        //     int temp = millis() - l_time;
        //     l_time = millis();
        //     if (temp > 0 && l_wait > temp)
        //     {
        //       l_wait -= temp;
        //     }
        //     else if (temp > 0)
        //     {
        //       active = false;
        //       stop();
        //     }
        //   }
        if (this->_timer.front())
        {
            this->stop();
        }
    }

    //defini quels resistance doive etre allumé
    //
    void open()
    {
        if (this->_open == false)
        {
            if (this->_debug == 1)
                Serial.println(F("trape Open"));
            this->_servo.write(TRAP_UP);
            Serial.println(this->_servo.read());
            Serial.println(digitalRead(PIN_TRAP_ALIM));
            digitalWrite(PIN_TRAP_ALIM, HIGH);
            Serial.println(digitalRead(PIN_TRAP_ALIM));
            this->_timer.reset();
            this->_timer.start(TRAP_OPEN_TIME);
            this->_open = true;
        }
    }

    void close()
    {
        if (this->_open == true)
        {
            if (this->_debug == 1)
                Serial.println(F("trape Down"));
            this->_servo.write(TRAP_DOWN);

            Serial.println(this->_servo.read());
            Serial.println(digitalRead(PIN_TRAP_ALIM));
            digitalWrite(PIN_TRAP_ALIM, HIGH);
            Serial.println(digitalRead(PIN_TRAP_ALIM));
            this->_timer.reset();
            this->_timer.start(TRAP_CLOSE_TIME);
            this->_open = false;
        }
    }

    void stop()
    {
        if (this->_debug == 1)
            Serial.println(F("trape Stop"));

        Serial.println(digitalRead(PIN_TRAP_ALIM));
        digitalWrite(PIN_TRAP_ALIM, LOW);
        Serial.println(digitalRead(PIN_TRAP_ALIM));
        this->_servo.write(TRAP_STOP);
        // Serial.println(this->_servo.read());
    }

    // CLI set parametre
    bool setparams(const char *params)
    {
        _params = params;
        return (params);
    }
    // CLI Execute
    bool execute(CLI &cli)
    {
        if (strcmp(_params, "_debug") == 0)
        {
            if (this->_debug == 1)
            {
                Serial.println(F("trape _debug Off"));
                this->_debug = 0;
            }
            else
            {
                Serial.println(F("trape _debug On"));
                this->_debug = 1;
            }
        }
        else if (strcmp(_params, "open") == 0)
        {
            Serial.println(F("Trape open"));
            this->open();
        }
        else if (strcmp(_params, "close") == 0)
        {
            Serial.println(F("trape close"));
            this->close();
        }

        // cli.print_P(PSTR("Autoradio "));
        // cli.println(_params);
        return false;
    }

    Servo _servo;

    uint8_t _debug;

private:
    bool _open;
    const char *_params;
    NeoTimer _timer;
    bool sleep;
    bool active;
    unsigned long int l_time;
    int l_wait;
    void *l_arg;
};

#endif