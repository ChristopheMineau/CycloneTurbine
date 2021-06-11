#ifndef CMI_SPEED_SELECTOR_H
#define CMI_SPEED_SELECTOR_H
#include <Arduino.h>
#include  "CMi_Buttons.h"
#include  "CMi_Constants.h"

/*******************************************************************************************/
class SpeedSelector {

  public:
    SpeedSelector(byte buttonPin, byte potPin); // button for range selection, pin for analog potentiometer reading
    bool update();
    int getSpeedRpm();
    int getSpeedRange();
    void setSpeedRange(byte s);
    
  private:
    Button button;
    byte currentSpeedRange;
    int currentSpeed;
    void ledDisplayUpdate();
    byte potPin;
};


#endif  // CMI_SPEED_SELECTOR_H
