#ifndef CMI_BUTTONS_H
#define CMI_BUTTONS_H
#include <Arduino.h>
#include  "CMi_Constants.h"



/*******************************************************************************************/
class Button {
  public:
    Button(byte buttonPin, bool activeLow=true);
    bool getState();
  private:
    const byte buttonPin;
    unsigned long initialTriggerTime;
    int triggerState;
};

#endif  // CMI_BUTTONS_H
