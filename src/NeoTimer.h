#ifndef NEOTIMER_H
#define NEOTIMER_H

#define NEOTIMER_INDEFINITE -1
#define NEOTIMER_UNLIMITED -1

#include <Arduino.h>

class NeoTimer
{
public:
    //Methods
    NeoTimer(unsigned long _t = 1000); //Constructor
    ~NeoTimer();                       //Destructor

    boolean done();  //Indicates time has elapsed
    boolean front(); //Returns true if transitions.
    boolean repeat(int times);
    boolean repeat(int times, unsigned long _t);
    boolean repeat();
    void repeatReset();
    boolean waiting();                                       // Indicates timer is started but not finished
    boolean started();                                       // Indicates timer has started
    void start(unsigned long times = NEOTIMER_INDEFINITE);   //Starts a timer
    void restart(unsigned long times = NEOTIMER_INDEFINITE); //Restarts a timer
    long stop();                                             //Stops a timer and returns elapsed time
    unsigned long elapsed();                                 //returns elapsed time
    long remaining();                                        //return remaining time
    void resume();
    void reset(); //Resets timer to zero
    void set(unsigned long t);
    unsigned long get();
    int repetitions = NEOTIMER_UNLIMITED;

private:
    struct myTimer
    {
        unsigned long time;
        unsigned long start;
        // long remaining;
        boolean started;
        boolean done;
    };

    struct myTimer _timer;
};

#endif
