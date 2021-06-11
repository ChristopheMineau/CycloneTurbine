#ifndef CMI_CONSTANTS_H
#define CMI_CONSTANTS_H

// Pins
#define DETECT 2           // zero cross detect pin
#define TACHO 3            // tacho signals input pin
#define REMOTE 4           // IR Remote control
#define RELAY 5            // relay pin
#define SPEED_BUTTON 6     // Speed button (INPUT_PULLUP)
#define LED_SPEED_3 7
#define LED_SPEED_2 8
#define LED_SPEED_1 9
#define LED_CAN_FULL 10    // Alarm LED Can is full
#define LED_DEFAULT 11     // Default LED Motor is stuck (no tacho signal)
#define LED_OK 12          // LED OK

#define SPEED_SET A2       // Analog Input Speed setting A2 16
#define GATE A3            // TRIAC gate pin 17
#define STOP_BUTTON A4     // STOP :(INPUT_PULLUP) 18
#define START_BUTTON A5    // START :(INPUT_PULLUP) 19
#define CAN_SENSOR A6      // Can fill IR sensor (active low,INPUT_PULLUP) 20

// Constants
#define TACHOPULSES 8      // number of pulses per revolution 
#define PULSE 4            // number of triac trigger pulse width counts. One count is 16 microseconds
#define DEBOUNCE_TIME 400  // in millis
#define BLINK_TIME 400     // in millis
#define STARTING_RPM 100   // starting RPM for soft start mode
#define POT_SPEED_RANGE 1500   // speed range corrsponding to the full pot adjustement range                                 
#define RANGE_1_MIN_SPEED 2000 // Start speed for Range1 (when pot to min)
                           // RANGE1 2000-3500 RANGE2 3500-5000 RANGE3 5000-6500       (motor shaft speed)

//#define DEBUG   // Activates the debug prints
#ifdef DEBUG
  #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...) void()
  #define DEBUG_PRINTLN(...) void()
#endif //    DEBUG

#endif //    CMI_CONSTANTS_HH
