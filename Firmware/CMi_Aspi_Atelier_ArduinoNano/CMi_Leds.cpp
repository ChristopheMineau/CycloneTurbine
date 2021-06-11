#include  "CMi_Leds.h"

Led::Led(byte ledPin) : ledPin(ledPin), state(OFF), blinkTime(0) {}

void Led::set() {
  state = ON;
  update();
}

void Led::reset() {
  state = OFF;
  update();
}

void Led::toggle() {
  if (state==ON)
    state = OFF;
  else if (state==OFF)
    state = ON;
  update();
}

void Led::blink() {
  if (not isBlinking()) {
    state = BLINKING_ON;
    blinkTime = millis();
    update();
  }
}

bool Led::isBlinking() {
  return (state == BLINKING_ON) or (state == BLINKING_OFF);
}

Led::State  Led::getState() {
  return state;
}

void  Led::update() {
  switch ( state ) {
    case ON:
      digitalWrite(ledPin, HIGH);
      break;
    case OFF:
      digitalWrite(ledPin, LOW);
      break;
    case BLINKING_ON:
      if ((millis() - blinkTime)>BLINK_TIME) {
        blinkTime = millis();
        state = BLINKING_OFF;
        update();
      } else 
        digitalWrite(ledPin, HIGH);
      break;
    case BLINKING_OFF:
      if ((millis() - blinkTime)>BLINK_TIME) {
        blinkTime = millis();
        state = BLINKING_ON;
        update();
      } else 
        digitalWrite(ledPin, LOW);
      break;
  }
}
