#ifndef CAV_H
#define CAV_H

#include <Arduino.h>
#include "debounce.h"
#include "config.h"
#include <CLI.h>



//gestion de la commande au volant
//la methode setup() doit etre appele dans la fonction d initialisation
//la methode  loop() doit etre appele dans chaque iteration de le fonction loop()
class CAV : CLI_Command
{
public:
    String data;
    //constructeur

    CAV(CLI &cli) : CLI_Command(cli,
                                PSTR("cav"),
                                PSTR("Commande au volant"),
                                PSTR("Usage:\t\tcav <commande>\n"
                                     "Where:\t<commande>\t debug, std"))
    // CAV()
    {
        stats = 0;
        cursor = 0;
        debounce = 0;
        _debug = 0;
    };

    //methode d intitialisation a apeler dans setup()
    void setup()
    {
        pinMode(CV_B1, OUTPUT); // set pin to output
        pinMode(CV_B2, OUTPUT); // set pin to output
        pinMode(CV_A2, OUTPUT); // set pin to output
        pinMode(CV_A1, INPUT);  // set pin to input
        pinMode(CV_A3, INPUT);  // set pin to input
        pinMode(CV_B3, INPUT);  // set pin to input

        digitalWrite(CV_A1, HIGH); // turn on pullup resistors /!\ logique inverse
        digitalWrite(CV_A3, HIGH); // turn on pullup resistors /!\ logique inverse
        digitalWrite(CV_B3, HIGH); // turn on pullup resistors /!\ logique inverse

        digitalWrite(CV_B1, HIGH); // /!\ logique inverse
        digitalWrite(CV_B2, HIGH); // /!\ logique inverse
        digitalWrite(CV_A2, HIGH); // /!\ logique inverse
        cursor = 0;
    }

    //methode de traitement a appeler dans chaque iteration de la boucle principal
    int loop()
    {

        static int old_stats = 0;
        bool a1 = !digitalRead(CV_A1);
        bool a3 = !digitalRead(CV_A3);
        bool b3 = !digitalRead(CV_B3);

        // Serial.print(F("a1 = "));
        // Serial.println(a1);
        // Serial.print(F("a3 = "));
        // Serial.println(a3);
        // Serial.print(F("b3 = "));
        // Serial.println(b3);

        // Serial.println(F(""));
        int loop_stats;

        loop_stats = a1 ? 1 : 0;
        // Serial.print(F("loop_stats = "));
        // Serial.println(loop_stats);
        loop_stats = a3 ? loop_stats + 2 : loop_stats;
        // Serial.print(F("loop_stats = "));
        // Serial.println(loop_stats);
        loop_stats = b3 ? loop_stats + 4 : loop_stats;

        loop_stats = loop_stats << cursor * 3;
        // Serial.print(F("loop_stats = "));
        // Serial.println(loop_stats);

        int sup = CV_MASK_CURSOR << cursor * 3;
        // sup = 0b1111111111 ^ sup;
        sup = 0b111111111 ^ sup;
        // Serial.print(F("sup = "));
        // Serial.println(sup);
        stats = stats & sup;
        // Serial.print(F("stats = "));
        // Serial.println(stats);
        stats = stats | loop_stats;

        if (stats != old_stats)
        {
            debounce = millis();
            old_stats = stats;
        }

        cursor = cursor == 2 ? 0 : cursor + 1;
        switch (cursor)
        {
        case 0: //CV_B1 (BTN_DOWN / VOL_DOWN / VOL_UP)

            Disable(CV_B2);
            Disable(CV_A2);
            UsePin(CV_B1);
            break;
        case 1: //CV_B2 (BTN_UP_RGT / BTN_UP_LFT)

            Disable(CV_A2);
            Disable(CV_B1);
            UsePin(CV_B2);
            break;
        case 2: //CV_A3 (molette)

            Disable(CV_B1);
            Disable(CV_B2);
            UsePin(CV_A2);
        }
    }

    // recuperation de l etat de l'ensemble des boutons avec anti-rebond
    int get_stats()
    {
        return Debounce(debounce, DEBOUNCE) ? stats : NONE;
    }

    int get_long_stats()
    {
        return Debounce(debounce, LONG_DEBOUNCE) ? stats : NONE;
    }

    // recuperation de l action a effectuer en fonction de l'etat de la commande au volant
    int get_action()
    {
        static int debug = NONE;
        static int molette = 0;
        int ret = NONE;
        unsigned long debounce_stats = get_stats();
        unsigned long debounce_long_stats = get_long_stats();

        if (_debug == 1)
        {
            Serial.print(F("cav = "));
            Serial.println(debounce_stats, BIN);
            Serial.println(debug);
        }

        int new_molette = debounce_stats & CV_MASK_MOL;
        if (new_molette != molette && ((new_molette == CV_MOL_1) || (new_molette == CV_MOL_2) || (new_molette == CV_MOL_3))) // si la molette a ete tournee
        {
            if (molette == CV_MOL_3 && new_molette == CV_MOL_1) // 3 -> 1
            {
                ret = AR_NEXT;
                if (_debug != 0 && debug != ret)
                    Serial.println(F("cav Next"));
                // Serial.println(F("Next"));
            }
            else if (molette == CV_MOL_1 && new_molette == CV_MOL_3) // 1 -> 3
            {
                ret = AR_PREV;
                if (_debug != 0 && debug != ret)
                    Serial.println(F("cav Prev"));
            }
            else if (new_molette > molette) // 1 -> 2 ou 2 -> 3
            {
                ret = AR_NEXT;
                if (_debug != 0 && debug != ret)
                    Serial.println(F("cav next"));
            }
            else if (new_molette < molette) // 3 -> 2 ou 2 -> 1
            {
                ret = AR_PREV;
                if (_debug != 0 && debug != ret)
                    Serial.println(F("cav prev"));
            }
            molette = new_molette;
        }
        else if ((debounce_stats & CV_VOL_UP_DOWN) == CV_VOL_UP_DOWN) // Bouton VOL+ et VOL-
        {
            ret = AR_MUTE;
            if (_debug != 0 && debug != ret)
                Serial.println(F("cav Mute"));
        }
        else if ((debounce_stats & CV_BTN_UP_LFT_RGT) == CV_BTN_UP_LFT_RGT) // Bouton haut droite et gauche
        {
            ret = AR_BAND;
            if (_debug != 0 && debug != ret)
                Serial.println(F("cav Band"));
        }
        else if ((debounce_stats & CV_BTN_UP_LFT_BTN_DOWN) == CV_BTN_UP_LFT_BTN_DOWN) // Bouton haut droite et boutons bas
        {
            ret = AR_DISPLAY;
            if (_debug != 0 && debug != ret)
                Serial.println(F("cav Display"));
        }
        else if (debounce_long_stats & CV_BTN_DOWN) // Bouton bas
        {
            ret = AR_SOURCE;
            if (_debug != 0 && debug != ret)
                Serial.println(F("cav Source"));
        }
        else if (debounce_long_stats & CV_VOL_DOWN) // Volume -
        {
            ret = AR_VOL_DOWN;
            if (_debug != 0 && debug != ret)
                Serial.println(F("cav Vol-"));
        }
        else if (debounce_long_stats & CV_VOL_UP) // Volume +
        {
            ret = AR_VOL_UP;
            if (_debug != 0 && debug != ret)
                Serial.println(F("cav Vol+"));
        }
        else if (debounce_long_stats & CV_BTN_UP_RGT) // Bouton haut droite
        {
            ret = AR_PRESET_UP;
            if (_debug != 0 && debug != ret)
                Serial.println(F("cav preset +"));
        }
        else if (debounce_long_stats & CV_BTN_UP_LFT) // Bouton haut gauche
        {
            ret = AR_PRESET_DOWN;
            if (_debug != 0 && debug != ret)
                Serial.println(F("cav preset-"));
        }

        debug = ret;
        return ret;
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
        if (strcmp(_params, "debug") == 0)
        {
            if (_debug == 1)
            {
                Serial.println(F("Debug = off"));
                _debug = 0;
            }
            else
            {
                Serial.println(F("Debug = on"));
                _debug = 1;
            }
        }
        else if (strcmp(_params, "display") == 0)
        {
            if (_debug == 2)
            {
                Serial.println(F("display = off"));
                _debug = 0;
            }
            else
            {
                Serial.println(F("display = on"));
                _debug = 2;
            }
        }

        // cli.print_P(PSTR("Autoradio "));
        // cli.println(_params);
        return false;
    }

    uint8_t _debug;

private:
    int stats;
    unsigned int cursor;
    unsigned long debounce;
    const char *_params;

    inline void UsePin(int Pin)
    {
        pinMode(Pin, OUTPUT); // set pin to output
        digitalWrite(Pin, LOW);
    }

    inline void Disable(int Pin)
    {
        pinMode(Pin, INPUT);    // set pin to input
        digitalWrite(Pin, LOW); // turn off pullup resistors
    }
};

#endif