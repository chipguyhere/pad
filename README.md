# cgh_EdgeInput.cpp

trying to put together my first world-usable Arduino library.  This is something I put together to use
AVR's "timer input capture" feature for high resolution capture of time-critical signals (only works on
specific pins), and "pin change interrupt" feature for medium resolution capture (works on many more pins).

Why?  More devices working reliably on the same Arduino project.  Better tolerance for interrupt latency.
It has worked well in my projects.


# working on the atmega4809 (Arduino Nano Every)

* Arduino seems to default to taking timer B3 for the millis() counter
* timer B3 only has one interrupt vector, TCB3_INT_vect, which is already taken
* timers B0,B1,B2 can be used for input capture.  Any timer can capture any pin.
* select an input pin by binding it to one of eight event capture channels
  * channels have limits on which pins they can bind to
  * example: channel 0 can only select pins from among portA and portB, same with channel 1.
  * channel 0,1: portA,portB (Arduino D2, D7, D9, D10, A4*, A5*) 
  * channel 2,3: portC,portD (Arduino D4, A0, A1, A2, A3, A6, A7)
  * channel 4,5: portE,portF (Arduino D3, D6, D8, D11, D12, D13, A4*, A5*)
  * channel 6,7 can't bind to pins
  * A4 and A5 appear twice because connected to two pins on the MCU (A4=PF2,PA2; A5=PF3,PA3)
  * from the correct CHANNEL structure, set GENERATOR to 0x40-0x47 (for capturing port A,C,E) and 
    0x48-0x4F (for capturing port B,D,F)
* route the channel to the "event input" of the timer we're using
* timing looks like it will be at 8MHz because Arduino doesn't set the system prescaler (so it's system clock)
  and our slowest least-impact option is CLK_PRE/2. another option is to capture output of TCA0 if free.
