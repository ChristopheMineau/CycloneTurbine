#define VERSION_CMI_ASPI_ATELIER "7.1"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <PID_v1.h>
#include <util/atomic.h>

#include  "CMi_Constants.h" /// !! Check this file for activating DEBUG prints
#include <IRremote.h>  // https://github.com/Arduino-IRremote/Arduino-IRremote   Version3.1.0
#include  "CMi_MotorControl.h"

// Globals
volatile unsigned int RPM = 0;      // real measured rpm variable (motor shaft)
volatile unsigned int tachoPulseCount = 0;    // tacho pulses count variable
unsigned long slidingSum = 0;       // sliding sum of the 16 last samples
unsigned long time_hist[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,}; // for sliding average
byte oldestIndex = 0;               // for sliding average computation : oldest element in the history buffer
unsigned int lastTachoCount = 0;         // additional tacho pulses count variable
unsigned long lastTachoStartCountingTime = 0;
unsigned long lastflash;
unsigned long lastpiddelay = 0;

const int sampleRate = 1;           // Variable that determines how fast our PID loop
const int rpmcorrection = 86;       // this requires that the actual rpm correspond to the measured ones
const int lcdinterval = 2000;       // lcd refresh interval in milliseconds
const int protection = 2000;        // protection will switch on when real rpm exceeds desired by value
const int debounceDelay = 50;       // the debounce time; increase if the output flickers
const int minoutputlimit = 80;      // limit of PID output
const int maxoutputlimit = 540;     // limit of PID output
const int mindimminglimit = 80;     // the shortest delay before triac fires
const int maxdimminglimit = 625;    // for 60Hz will be 520
const int  risetime = 1000;   //100  // RPM rise time delay in microseconds ( x RPM) (takes 5s to reach 1000rpm) 

int dimming = 540;                   // this should be the same as maxoutputlimit
int desiredRPM = 0;
bool desiredMotorState = false;
int intermediateRpm = STARTING_RPM;
byte relayState = LOW;              // the current state of the relay pin
bool softStartRunningMode = false;  // flag for soft start running mode
bool motorStarted = false;          // flag for enabling Triac triggering
bool normalRunningMode = false;     // flag for nominal motor running mode

double Setpoint, Input, Output;       // define PID variables
double sKp = 0.1, sKi = 0.2, sKd = 0; // PID tuning parameters for starting motor
double rKp = 0.25, rKi = 1, rKd = 0;  // PID tuning parameters for runnig motor

PID myPID(&Input, &Output, &Setpoint, sKp, sKi, sKd, DIRECT); // define PID variables and parameters
MotorControl motorStatus = MotorControl();
  
void setup() {

// Just to know which program is running on my Arduino
  Serial.begin(115200);
  Serial.println(F("START " __FILE__ " from " __DATE__ " version " VERSION_CMI_ASPI_ATELIER));

#ifndef DEBUG
  Serial.end();
#endif   // DEBUG

  // Set up pins
  // Input digital pins
  pinMode(REMOTE, INPUT);              // IR Remote control
  pinMode(CAN_SENSOR, INPUT);          // Can full IR sensor
  pinMode(SPEED_BUTTON, INPUT_PULLUP); // set the button pin
  pinMode(DETECT, INPUT);              // set the zero cross detect pin
  pinMode(TACHO, INPUT);               // set the tacho pulses detect pin
  pinMode(START_BUTTON, INPUT_PULLUP); // set the start button
  pinMode(STOP_BUTTON, INPUT_PULLUP);  // set the stop button
  pinMode(CAN_SENSOR, INPUT_PULLUP);   // set the stop button  !! A6 and A7 are only analog pins on Nano, must use analogRead to read them.

  // Input Analog pins
  pinMode(SPEED_SET, INPUT); // analog SPEED_SET A2  

  // Output digital pins
  pinMode(LED_SPEED_1, OUTPUT);       // set the speed range indicator led pins
  pinMode(LED_SPEED_2, OUTPUT);
  pinMode(LED_SPEED_3, OUTPUT);
  pinMode(RELAY, OUTPUT);             // set the relay  pin
  pinMode(GATE, OUTPUT);              // set the TRIAC gate control pin
  pinMode(LED_OK, OUTPUT);            // LED OK
  pinMode(LED_CAN_FULL, OUTPUT);      // Alarm LED Can is full
  pinMode(LED_DEFAULT, OUTPUT);       // Default LED Motor is stuck (no tacho signal)

  // Remote control 
  IrReceiver.begin(REMOTE, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
  // Initial relay state
   digitalWrite(RELAY, relayState);    // initialize relay output
  // Pid Initial values
   Input = 200;                        // asiign initial value for PID
   Setpoint = 200;                     // asiign initial value for PID
  // turn the PID on
   myPID.SetMode(AUTOMATIC);
   myPID.SetOutputLimits(minoutputlimit, maxoutputlimit);
   myPID.SetSampleTime(sampleRate);    // Sets the sample rate

  // set up Timer1
   OCR1A = 100;                        // initialize the comparator
   TIMSK1 = 0x03;                      // enable comparator A and overflow interrupts
   TCCR1A = 0x00;                      // timer control registers set for
   TCCR1B = 0x00;                      // normal operation, timer disabled

  // set up zero crossing interrupt IRQ0 on pin 2.
  attachInterrupt(digitalPinToInterrupt(DETECT), zeroCrossingInterrupt, RISING); // 0
  // set up tacho sensor interrupt IRQ1 on pin3
  attachInterrupt(digitalPinToInterrupt(TACHO), tacho, FALLING);   // 1
}


// Interrupt Service Routines
void zeroCrossingInterrupt() { // zero cross detect
  TCCR1B = 0x04;               // start timer with divide by 256 input
  TCNT1 = 0;                   // reset timer - count from zero
  OCR1A = dimming;             // set the compare register brightness desired.
  digitalWrite(GATE, LOW);     // for security, if ever we missed the overflow interrupt
}


ISR(TIMER1_COMPA_vect) {       // comparator match // see OCR1A value
  if (motorStarted == true) {     // flag for start up delay
    digitalWrite(GATE, HIGH);  // set TRIAC gate to high
    TCNT1 = 65536 - PULSE;     // trigger pulse width
  }
}

ISR(TIMER1_OVF_vect) {         // timer1 overflow
  digitalWrite(GATE, LOW);     // turn off TRIAC gate
  TCCR1B = 0x00;               // disable timer stops unintended triggers
}

// RPM counting routine
// The RPM is computing based on a sliding average of 16 consecutive measures.
// This filters any possible spark or false falling edge detection on the tacho circuit.
// As a consequence, the 16 first calls of the routine will deliver a too high
// RPM value.
void tacho() {
  tachoPulseCount++;
  unsigned long thistime = micros() - lastflash;
  slidingSum = thistime + slidingSum - time_hist[oldestIndex];  // see http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  time_hist[oldestIndex] = thistime;
  oldestIndex ++;
  oldestIndex &= 0x0F;  // circular buffer of 16 elements
  unsigned long revPeriod =  slidingSum>>1 ;   // average = slidingSum/16 TACHOPULSE=8   => revolutionPeriod = slidingSum/2 in micro secondes
  RPM = 60  * 1000000 / revPeriod;
  lastflash = micros();
}

namespace DEBUG {
  void displayStatus() {
#ifndef DEBUG                    // See CMi_Constants.h for activating the DEBUG flag
    return;
#endif   // DEBUG
    static unsigned long lastTime = 0;
    if ((millis() - lastTime )> 3000) {
      DEBUG_PRINTLN();
      DEBUG_PRINT("Motor state = ");
      DEBUG_PRINTLN(motorStatus.on);
      DEBUG_PRINT("relayState = ");
      DEBUG_PRINTLN(relayState);
      DEBUG_PRINT("RPM = ");
      DEBUG_PRINTLN(RPM); 
      DEBUG_PRINT("desiredRPM = ");
      DEBUG_PRINTLN(desiredRPM); 
      DEBUG_PRINT("normalRunningMode = ");
      DEBUG_PRINTLN(normalRunningMode);
      DEBUG_PRINT("softStartRunningMode = ");
      DEBUG_PRINTLN(softStartRunningMode); 
      DEBUG_PRINT("motorStarted = ");
      DEBUG_PRINTLN(motorStarted);
      lastTime = millis();
    }  
  }
  
  void displayMotorSpeed() {
#ifndef DEBUG
    return;
#endif   // DEBUG
    static unsigned long lastTime = 0;
    if ((millis() - lastTime )> 3000) {
      DEBUG_PRINT("Motor speed = ");
      DEBUG_PRINTLN(motorStatus.speedSelector.getSpeedRpm());
      lastTime = millis();
    }
  }
  
  void displayCanFullState() {
#ifndef DEBUG
    return;
#endif   // DEBUG
    static unsigned long lastTime = 0;
    if ((millis() - lastTime )> 3000) {
      DEBUG_PRINT("Can Full Alarm = ");
      DEBUG_PRINTLN( (motorStatus.canFullAlarm)?  "ON" :  "OFF");
      DEBUG_PRINT("PIN A6 = ");
      DEBUG_PRINTLN( analogRead(A6));
      lastTime = millis();
    }
  }
}

int checkRemoteControl() {
  int irCode = 0xFF;
  static int currentlyReceivingCode = 0xFF;
  int receivedCode;
  static unsigned long lastIrChangeTime = 0;
  
  if (IrReceiver.decode()) {
     IrReceiver.resume(); // Enable receiving of the next value
     receivedCode = IrReceiver.decodedIRData.command; 
    
   // if (millis() - lastIrChangeTime > DEBOUNCE_TIME) {
    if (receivedCode != currentlyReceivingCode) {           // reject repetitions of the same code
       if ((IrReceiver.decodedIRData.protocol == RC5) and   // Protocol used by the Philips VCR remote control
           ((receivedCode == 0xc) or
            (receivedCode == 0x1) or
            (receivedCode == 0x2) or
            (receivedCode == 0x3))) {
#ifdef DEBUG
          IrReceiver.printIRResultShort(&Serial);  // FOR DEBUG 
          DEBUG_PRINTLN(IrReceiver.decodedIRData.flags);
#endif   // DEBUG

            currentlyReceivingCode = receivedCode;
            irCode = receivedCode;
            lastIrChangeTime = millis();
        }   
    }     
  }
  if (millis() - lastIrChangeTime > DEBOUNCE_TIME)
     currentlyReceivingCode = 0xFF;
  
  return irCode;
}

bool hasDesiredMototStateChanged(bool newState) {
  static bool lastState = false;
  bool resu = (newState != lastState);
  lastState =  newState;
  return resu;
}

bool hasSpeedRangeChanged(int newRange) {
  static int lastSpeedRange = 1;
  bool resu = (lastSpeedRange != newRange);
  lastSpeedRange =  newRange;
  return resu;
}

void setSoftStartInitialSpeed() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    intermediateRpm = RPM;
  }
  if (intermediateRpm < STARTING_RPM) 
    intermediateRpm = STARTING_RPM;
}

// Error Handlers
void stuckError() {   // The tachometer reports no rotation despite the desired motor status which is ON
  DEBUG_PRINTLN("### Stuck Error. No Tachometer signal detected...");
  motorStatus.setMotorDefault();
  while (1) {                  // Dead end ...
    motorStatus.defaultLed.update();
    motorStatus.canFullLed.update();
    delay(100);
  } 
}

void exceedError() {
  DEBUG_PRINTLN("### Speed Exceeded Error. Maybe the Triac is damaged...");
  motorStatus.setMotorDefault(true);
  while (1) {                  // Dead end ...
    motorStatus.defaultLed.update();
    motorStatus.canFullLed.update();
    delay(100);
  }
}

void loop() {
  //  DEBUG::displayMotorSpeed();
  DEBUG::displayStatus();
  //  DEBUG::displayCanFullState();
  
  // Get the last Faceplate controls status and update faceplate Led display
  motorStatus.update(checkRemoteControl());
  
  // Set the target RPM speed
  desiredRPM = motorStatus.speedSelector.getSpeedRpm();
  
  // check the start / stop commands
  desiredMotorState = motorStatus.on; // ex  buttonState
  if (hasDesiredMototStateChanged(desiredMotorState)) {
    if (desiredMotorState) {           // motor startup
      relayState = HIGH;
      softStartRunningMode = true;
      normalRunningMode = false;
      setSoftStartInitialSpeed();
      digitalWrite(RELAY, relayState);   // set the Relay ON
      delay (300);                       // delay to prevent sparks on relay contacts
      motorStarted = true;               // flag to start trigerring the Triac
    } else {                           // motor chutdown
      Setpoint = 200;
      Input = 200;
      softStartRunningMode = false;
      normalRunningMode = false;
      motorStarted = false;              // no more Triac trigerring
      delay (300);                       // delay to prevent sparks on relay contacts
      relayState = LOW;
      digitalWrite(RELAY, relayState);   // set the Relay OFF
    }
  }

  // Check if speed range has been changed while running
  if (hasSpeedRangeChanged(motorStatus.speedSelector.getSpeedRange()) and normalRunningMode) {
      DEBUG_PRINTLN("Speed range has changed : returning to softsatart mode.");
      softStartRunningMode = true;
      normalRunningMode = false;   
      setSoftStartInitialSpeed();
  }

  //soft start mode : when big speed change is demanded : startup or speed range change while running
  if (softStartRunningMode == true) {       // Note that we do not loop and check IR or buttons during softstart phase
    motorStatus.okLed.blink();
    myPID.SetTunings(sKp, sKi, sKd);        // Set the PID gain constants and start
    int speedDelta, speedStep;
    if (desiredRPM > intermediateRpm) {     // acceleration
      speedDelta = desiredRPM - intermediateRpm;
      speedStep = +1;
    } else {                                // deceleration
      speedDelta = intermediateRpm - desiredRPM;
      speedStep = -1;
    }
    for (int d = 1; d <= speedDelta; d++) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        Input = RPM;
      }  
      Setpoint = intermediateRpm;
      myPID.Compute();
      dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // inverse the output
      dimming = constrain(dimming, mindimminglimit, maxdimminglimit);     // check that dimming is in 20-625 range
      intermediateRpm += speedStep;
      delayMicroseconds (risetime);
      // continue updating things during the soft start look which may be long
      DEBUG::displayStatus();
      motorStatus.defaultLed.update();
      motorStatus.canFullLed.update();
      motorStatus.okLed.update();
    }
    if (intermediateRpm == desiredRPM) {
      lastTachoStartCountingTime = millis();
      lastpiddelay = millis();
      softStartRunningMode = false;
      normalRunningMode = true;
      motorStatus.okLed.set();
    }
  }

  // normal motor running mode
  if (relayState == HIGH && softStartRunningMode == false) {
    myPID.SetTunings(rKp, rKi, rKd);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      Input = RPM;
    }
    Setpoint = desiredRPM;
    myPID.Compute();
    dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // reverse the output
    dimming = constrain(dimming, mindimminglimit, maxdimminglimit);     // check that dimming is in 20-625 range
  }

  // diagnose a fault and turn on protection
  unsigned long tachoStartCountingTime = millis();
  if (tachoStartCountingTime - lastTachoStartCountingTime >= 1000) {
    unsigned int tachoCount; 
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      tachoCount = tachoPulseCount;
    }
    if (tachoCount == 0 && relayState == HIGH && normalRunningMode == true) {
      motorStarted = false;         // flag to turn off triac before relay turns off
      delay (300);                  // delay to prevent sparks on relay contacts
      digitalWrite(RELAY, LOW);
      relayState = LOW;
      stuckError();                 // dead end
    }
    lastTachoCount = tachoCount;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      tachoPulseCount = 0;
    }
    lastTachoStartCountingTime = millis();
  }

  //reset rpm after motor stops
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (tachoPulseCount == 0 && relayState == LOW) {
      RPM = 0;
    }
  }

  // protection against high rpm. i e triac damage
  unsigned int currentRpm;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currentRpm = RPM;
  }
  if (relayState == HIGH && (currentRpm > desiredRPM + protection) && softStartRunningMode == false) {
    motorStarted = false;            // flag to turn off triac before relay turns off
    delay (300);                     // delay to prevent sparks on relay contacts
    digitalWrite(RELAY, LOW);
    relayState = LOW;
    exceedError();
  }

}
