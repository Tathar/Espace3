#ifndef NEODEBOUNCE_H
#define NEODEBOUNCE_H

#include "NeoTimer.h"

class NeoDebounce
{
public:
    //Methods
    NeoDebounce(long unsigned int times = 50); //Constructor

    boolean stats(boolean stats); //Indicates debonced stats
    boolean front(boolean stats); //Returns true at debounced transitions stats.
    boolean fell(boolean stats);  //Returns true if stats debounced transitions from True to False.
    boolean rose(boolean stats);  //Returns true if stats debounced transitions from False to True.

private:
    NeoTimer _timer;
    boolean _old_stats;
    boolean _debounced_stats;
    boolean first;
};

#endif
