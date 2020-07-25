#ifndef AUTORADIO_H
#define AUTORADIO_H

#include <Arduino.h>
#include "NeoTimer.h"
#include <CLI.h>
#include <SPI.h>

class AutoRadio : CLI_Command
{
public:
    AutoRadio(CLI &cli);

    void setup();

    void loop();

    void
    set_audio(int val);

    void set_FP(bool value);

    void set_MA(bool value);

    // CLI set parametre
    bool setparams(const char *params);
    // CLI Execute
    bool execute(CLI &cli);

private:
    const char *_params;
    uint8_t _debug;
    NeoTimer _timer;
    void set_pot(int val);
};

#endif