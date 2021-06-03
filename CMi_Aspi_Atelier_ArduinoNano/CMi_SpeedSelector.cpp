#include  "CMi_SpeedSelector.h"

SpeedSelector::SpeedSelector(byte buttonPin, byte potPin): button(Button(buttonPin)), potPin(potPin), currentSpeedRange(1) {}

bool SpeedSelector::update(){
  bool changed = false;
  if (button.getState()) {
    currentSpeedRange = (currentSpeedRange==3) ? 1 : currentSpeedRange+1 ;
    changed = true;
  }
  int curSpeed = currentSpeed;
  currentSpeed = RANGE_1_MIN_SPEED + map(analogRead(potPin), 0, 1023, 0, POT_SPEED_RANGE) +   (currentSpeedRange-1) * POT_SPEED_RANGE ;
  if (currentSpeed != curSpeed) 
    changed = true;
    
  ledDisplayUpdate();
  
  return changed;
}

void SpeedSelector::ledDisplayUpdate(){
  if (currentSpeedRange == 1) 
    digitalWrite(LED_SPEED_1, HIGH);
  else
    digitalWrite(LED_SPEED_1, LOW);
  if (currentSpeedRange == 2) 
    digitalWrite(LED_SPEED_2, HIGH);
  else
    digitalWrite(LED_SPEED_2, LOW);
  if (currentSpeedRange == 3) 
    digitalWrite(LED_SPEED_3, HIGH);
  else
    digitalWrite(LED_SPEED_3, LOW);
}

int SpeedSelector::getSpeedRpm() {
  return currentSpeed;
}

int SpeedSelector::getSpeedRange() {
  return currentSpeedRange;
}

void SpeedSelector::setSpeedRange(byte s) {
  currentSpeedRange = s;
  currentSpeed = analogRead(potPin) * currentSpeedRange ;
}
