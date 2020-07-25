
#include "NeoDebounce.h"

NeoDebounce::NeoDebounce(long unsigned int times) //Constructor
{
    this->_timer.set(times);
    this->first = true;
};

boolean NeoDebounce::stats(boolean stats) //Indicates debonced stats
{
    if (this->first)
    {
        this->_old_stats = stats;
        this->_debounced_stats = stats;
        this->first = false;
        return stats;
    }

    if (stats != this->_old_stats)
    {
        this->_old_stats = stats;
        this->_timer.reset();
        this->_timer.start();
        return !stats;
    }

    if (stats == this->_old_stats)
    {
        if (this->_timer.done())
            return stats;
        else
            return !stats;
    }

    return stats;
}

boolean NeoDebounce::front(boolean stats) //Returns true at debounced transitions stats.
{
    boolean debounced_stats = this->stats(stats);
    if (this->_debounced_stats != debounced_stats)
    {
        this->_debounced_stats = debounced_stats;
        return true;
    }
    return false;
}
boolean NeoDebounce::fell(boolean stats) //Returns true if stats debounced transitions from True to False.
{
    if (this->front(stats) && this->_debounced_stats == false)
        return true;

    return false;
}

boolean NeoDebounce::rose(boolean stats) //Returns true if stats debounced transitions from False to True.
{
    if (this->front(stats) && this->_debounced_stats == true)
        return true;

    return false;
}