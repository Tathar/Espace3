
#include "AutoRadio.h"
#include "config.h"

AutoRadio::AutoRadio(CLI &cli) : CLI_Command(cli,
                                             PSTR("ar"),
                                             PSTR("Auto Radio"),
                                             PSTR("Usage:\t\tAR <commande>\n"
                                                  "Where:\t<commande>\t vol+, vol-, mute, ..."))
{
    _debug = 0;
    _params = nullptr;
};

void AutoRadio::setup()
{
    //audio
    pinMode(AR_TIP, OUTPUT);    // set pin to output
    pinMode(AR_RING, OUTPUT);   // set pin to output
    digitalWrite(AR_TIP, LOW);  //logique inverse
    digitalWrite(AR_RING, LOW); //logique inverse

    //relais marche arriere et frein de parc
    pinMode(PIN_AR_FP, OUTPUT);    // set pin to output
    pinMode(PIN_AR_MA, OUTPUT);    // set pin to output
    digitalWrite(PIN_AR_FP, HIGH); //logique inverse
    digitalWrite(PIN_AR_MA, HIGH); //logique inverse
}

void AutoRadio::loop()
{
    if (this->_timer.front())
    {
        digitalWrite(AR_TIP, LOW);
        digitalWrite(AR_RING, LOW);
        if (_debug == 1)
        {
            Serial.print(F("None time = "));
            Serial.println(this->_timer.elapsed());
        }
    }
}

void AutoRadio::set_audio(int val)
{
    //static int old_val = 0;
    // if (old_val != val)
    {
        // if (val == NONE)
        // {
        //   digitalWrite(AR_TIP, HIGH);
        //   digitalWrite(AR_RING, HIGH);
        // }
        // else
        if (val < AR_NEED_RING && val != NONE)
        {
            digitalWrite(AR_TIP, LOW);
            digitalWrite(AR_RING, LOW);
            set_pot(val);
            digitalWrite(AR_TIP, HIGH);
            this->_timer.reset();
            this->_timer.start(100);
            if (_debug == 1)
            {
                Serial.print(F("set "));
                Serial.println(val);
            }
        }
        else if (val >= AR_NEED_RING && val != NONE)
        {
            digitalWrite(AR_TIP, LOW);
            digitalWrite(AR_RING, LOW);
            set_pot(val - AR_NEED_RING);
            digitalWrite(AR_TIP, HIGH);
            digitalWrite(AR_RING, HIGH);

            this->_timer.reset();
            this->_timer.start(100);
            if (_debug == 1)
            {
                Serial.print(F("set "));
                Serial.println(val);
            }
        }
        //old_val = val;
    }
}

void AutoRadio::set_FP(bool value)
{
    if (value)
    {
        digitalWrite(PIN_AR_FP, HIGH);
    }
    else
    {
        digitalWrite(PIN_AR_FP, LOW);
    }
}

void AutoRadio::set_MA(bool value)
{
    if (value)
    {
        digitalWrite(PIN_AR_MA, LOW);
    }
    else
    {
        digitalWrite(PIN_AR_MA, HIGH);
    }
}

// CLI set parametre
bool AutoRadio::setparams(const char *params)
{
    _params = params;
    return (params);
}

// CLI Execute
bool AutoRadio::execute(CLI &cli)
{
    if (strcmp(_params, "debug") == 0)
    {
        if (_debug == 1)
        {
            cli.println(F("Debug = off"));
            _debug = 0;
        }
        else
        {
            cli.println(F("Debug = on"));
            _debug = 1;
        }
    }
    if (strcmp(_params, "none") == 0)
    {
        cli.println(F("action = NONE"));
        set_audio(NONE);
    }
    if (strcmp(_params, "band") == 0)
    {
        cli.println(F("action = BAND"));
        set_audio(AR_BAND);
    }
    if (strcmp(_params, "display") == 0)
    {
        cli.println(F("action = DISPLAY"));
        set_audio(AR_DISPLAY);
    }
    if (strcmp(_params, "mute") == 0)
    {
        cli.println(F("action = MUTE"));
        set_audio(AR_MUTE);
    }
    if (strcmp(_params, "next") == 0)
    {
        cli.println(F("action = NEXT"));
        set_audio(AR_NEXT);
    }
    if (strcmp(_params, "preset-") == 0)
    {
        cli.println(F("action = Preset down"));
        set_audio(AR_PRESET_DOWN);
    }
    if (strcmp(_params, "preset+") == 0)
    {
        cli.println(F("action = preset UP"));
        set_audio(AR_PRESET_UP);
    }
    if (strcmp(_params, "prev") == 0)
    {
        cli.println(F("action = prev"));
        set_audio(AR_PREV);
    }
    if (strcmp(_params, "source") == 0)
    {
        cli.println(F("action = source"));
        set_audio(AR_SOURCE);
    }
    if (strcmp(_params, "vol-") == 0)
    {
        cli.println(F("action = vol -"));
        set_audio(AR_VOL_DOWN);
        // trape.down();
    }
    if (strcmp(_params, "vol+") == 0)
    {
        cli.println(F("action = Vol +"));
        set_audio(AR_VOL_UP);
    }

    if (strcmp(_params, "ma") == 0)
    {
        digitalWrite(PIN_AR_MA, !digitalRead(PIN_AR_MA));
        cli.print(F("marche_arriere = "));
        cli.println(digitalRead(PIN_AR_MA));
    }

    if (strcmp(_params, "fp") == 0)
    {
        digitalWrite(PIN_AR_FP, !digitalRead(PIN_AR_FP));
        cli.print(F("frein_parking = "));
        cli.println(digitalRead(PIN_AR_FP));
    }

    // cli.print_P(PSTR("Autoradio "));
    // cli.println(_params);
    return false;
}

void AutoRadio::set_pot(int val)
{
    val = constrain(val, 0, 255);
    // set the CS pin to low to select the chip:
    digitalWrite(PIN_SPI_CS_DP, LOW);
    // send the command and value via SPI:
    // SPI.transfer(POT1_SEL);
    // SPI.transfer(val);
    SPI.transfer(BOTH_POT_SEL);
    SPI.transfer(val);
    // Set the CS pin high to execute the command:
    digitalWrite(PIN_SPI_CS_DP, HIGH);
}
