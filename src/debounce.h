#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <Arduino.h>

inline bool Debounce(unsigned long time, unsigned long debounce = DEBOUNCE)
{
    return (time + debounce) < millis() ? true : false;
}
#endif