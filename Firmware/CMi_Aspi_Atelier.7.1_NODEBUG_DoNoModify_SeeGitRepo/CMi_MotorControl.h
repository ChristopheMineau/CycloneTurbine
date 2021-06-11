#ifndef CMI_MOTOR_CONTROL_H
#define CMI_MOTOR_CONTROL_H
#include <Arduino.h>
#include  "CMi_Constants.h"
#include  "CMi_Buttons.h"
#include  "CMi_Leds.h"
#include  "CMi_SpeedSelector.h"

/*******************************************************************************************/

class MotorControl {
  public:
    bool on;                // motor turned on or not
    bool canFullAlarm;      // can is full 
    bool defaultAlarm;      // motor default
    bool default2Alarm;     // motor default (highSeverity)
    
    MotorControl();
    void update(int irCode);
    void setMotorDefault(bool highSeverity = false);
    SpeedSelector speedSelector;  // keeps speed range and speed rpm, updates faceplate controls
    Led defaultLed;          // need to update it during dead end loops, so public
    Led canFullLed;          // need to update it during dead end loops, so public
    Led okLed;               // need to update it during dsoft start loop, so public
    
  private:
    Button buttonStart;
    Button buttonStop;

    
    void inputUpdate();
    void updateDisplay();
};


#endif  // CMI_MOTOR_CONTROL_H
