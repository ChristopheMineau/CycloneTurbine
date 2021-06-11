#ifndef CMI_LEDS_H
#define CMI_LEDS_H
#include <Arduino.h>
#include  "CMi_Constants.h"

/*******************************************************************************************/
class Led {
  public:
    Led(byte ledPin);
    void set();
    void blink();
    void reset();
    void toggle();
    enum State { ON, OFF, BLINKING_ON, BLINKING_OFF } ;
    State getState();
    void update();
    bool isBlinking();
    
  private:
    byte ledPin;
    State state;
    unsigned long blinkTime;
};


#endif  // CMI_LEDS_H
