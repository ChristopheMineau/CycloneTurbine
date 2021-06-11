#include  "CMi_Buttons.h"

Button::Button(byte buttonPin, bool activeLow) : buttonPin(buttonPin), initialTriggerTime(0) {
  triggerState = activeLow ? LOW : HIGH;  
}

bool Button::getState() {
  if (digitalRead(buttonPin)==triggerState) {
     unsigned long currentTime = millis();
    if (initialTriggerTime == 0) 
      initialTriggerTime = currentTime;
    if ((currentTime - initialTriggerTime) > DEBOUNCE_TIME) {
      initialTriggerTime = currentTime;
      return true;
    } else
      return false;
      
  } else {
    initialTriggerTime = 0;
    return false;
  }
}
