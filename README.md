# cgh_EdgeInput.cpp

trying to put together my first world-usable Arduino library.  This is something I put together to use
AVR's "timer input capture" feature for high resolution capture of time-critical signals (only works on
specific pins), and "pin change interrupt" feature for medium resolution capture (works on many more pins).

Why?  More devices working reliably on the same Arduino project.  Better tolerance for interrupt latency.
It has worked well in my projects.


# working on the atmega4809 (Arduino Nano Every)

* Arduino seems to default to taking timer B3 for the millis() counter
* timer B3 only has one interrupt vector, TCB3_INT_vect, which is already taken
* timers B0,B1,B2 can be used for input capture
