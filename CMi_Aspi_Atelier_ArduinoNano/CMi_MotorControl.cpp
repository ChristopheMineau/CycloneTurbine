#include  "CMi_MotorControl.h"


MotorControl::MotorControl() : 
        on(false), 
        canFullAlarm(false), 
        defaultAlarm(false), 
        speedSelector(SpeedSelector(SPEED_BUTTON, SPEED_SET)),
        buttonStart(Button(START_BUTTON)), 
        buttonStop(Button(STOP_BUTTON)),
        okLed(Led(LED_OK)),
        canFullLed(Led(LED_CAN_FULL)),
        defaultLed(Led(LED_DEFAULT))
        {}

void MotorControl::update(int irCode) {
  // handle irCode if any
  switch (irCode) {
    case 0xC:     // ON OFF button  on PHILIPS VCR remote control
      on = not on;
      DEBUG_PRINT("ON OFF button :");
      DEBUG_PRINTLN(on);
      break;
    case 0x1:     //  button 1  on PHILIPS VCR remote control
      speedSelector.setSpeedRange(1);
      break;
    case 0x2:     //  button 2  on PHILIPS VCR remote control
      speedSelector.setSpeedRange(2);
      break;
    case 0x3:     //  button 3  on PHILIPS VCR remote control
      speedSelector.setSpeedRange(3);
      break;
  }
  inputUpdate();
  updateDisplay();
}

void MotorControl::setMotorDefault(bool highSeverity) { 
  on=false;
  okLed.reset();
  if (highSeverity) {
    default2Alarm = true;
    defaultAlarm = false;
    defaultLed.blink();
  } else {
    default2Alarm = false;
    defaultAlarm = true;
    defaultLed.set();
  }
}

void MotorControl::inputUpdate() {
    if (buttonStart.getState()) {
      on = true;
    }
      
    if (buttonStop.getState()) {
      on = false;
    }

    canFullAlarm = (analogRead(CAN_SENSOR)<500);
    speedSelector.update();

    if (on)            okLed.set(); else okLed.reset();
    if (canFullAlarm)  canFullLed.blink(); else canFullLed.reset();
    if (defaultAlarm)  defaultLed.set(); else defaultLed.reset();
    if (default2Alarm) defaultLed.blink(); else defaultLed.reset();
}



void MotorControl::updateDisplay() {
  okLed.update();
  canFullLed.update();
  defaultLed.update();
}
