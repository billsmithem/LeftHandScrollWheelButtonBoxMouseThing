#include <Arduino.h>

/*
	Modified to support multiple modes of operation
		- Buttons + Mouse wheel
		- Buttons + Left & Right arrow keys (useful for video editing)
	Modified to use buxtronix rotary encoder library
	Added debugging switch - reset while holding down thumb button
  Added analog thumbstick
*/

#include <Keyboard.h>
#include <Mouse.h>
#include "Rotary.h"
#include "version.h"

// #define	DEBUGGING
// #define DEBUG_MODES
//#define DEBUG_ENCODER
#define DEBUG_BUTTONS
// #define DEBUG_BLINK

// Arduino pin definitions
#define ENCODER_CLOCK		2		// Rotary encoder clock
#define ENCODER_DATA		3		// Rotary encoder data 
#define ENCODER_SWITCH 		A1		// rotary encoder switch
#define LEFT_SWITCH			5		// left mouse button
#define	RIGHT_SWITCH		6		// right mouse button
#define THUMB_SWITCH		7		// pin for thumb switch

#define EXTERN_LED			16		// external LED

// keys
#define	THUMB_KEY			KEY_LEFT_CTRL	// thumb key value

// #define DEBOUNCE_TIME		100		// delay 100 milliseconds for switch transitions
#define DEBOUNCE_TIME		20		// delay 20 milliseconds for switch transitions

#define LED_PULSETIME		250
#define DEADTIME			1000	// delay between LED pulse streams

bool debug_mode = false;			// start in normal mode unless thumb button pressed during startup

void blink(int, int);
void print_filename();
void externLed(int);

/*
	Setup mode
		Toggle setup mode by holding down buttons 1 & 2, then pressing button 3
*/
// what mode are we in? allows functionality changes
bool setupMode = false;		// changing modes
// available modes
enum hidMode {
	mouse,			// encoder sends mouse wheel events
	arrows			// encoder sends left and right cursor key events (useful for video editing)
} mode = mouse;

void nextMode() {
	switch (mode) {
		case mouse:
			mode = arrows;
			Serial.println("mode == ARROWS");
			break;
		case arrows:
		default:		// something is broken, switch back to mouse
			mode = mouse;
			Serial.println("mode == MOUSE");
			break;
	}
}

void displayMode() {	// blink LED to display mode we're in
	switch (mode) {
		case mouse:
			blink(1, LED_PULSETIME);
			break;
		case arrows:
			blink(2, LED_PULSETIME);
			break;
		default:		// error!!!
			blink(10, LED_PULSETIME);
	}
}

class Button {
	private:
		int pin;
		String buttonName;
		int state;
		int clickVal;
		bool isButton;

	// constructor
	public:
		Button(int pinNum, String name, int click, bool buttonMode = true ) {
			pin = pinNum;
			state = HIGH;
			buttonName = name;
			clickVal = click;
			isButton = buttonMode;
			pinMode(pin, INPUT_PULLUP);
		}

		int State() {
			return state;
		}

		// bool Update() {
		void Update() {
			int buttonState = digitalRead(pin);
			if (buttonState != state) {			// state change?
				delay(DEBOUNCE_TIME);
				buttonState = digitalRead(pin);
				if (buttonState != state) {		// if really changed
					state = buttonState;
					if (debug_mode)
						Serial.print(buttonName);
					if (!setupMode) {
						if (state == LOW) {
							if (debug_mode)
								Serial.println(" down");
							if (isButton)
								Mouse.press(clickVal);
							else {
								if (debug_mode)
									Serial.println(" key pressed");
								Keyboard.press(clickVal);
							}
						} else {
							if (debug_mode)
								Serial.println(" up");
							if (isButton)
								Mouse.release(clickVal);
							else {
								if (debug_mode)
									Serial.println(" key released");
								Keyboard.release(clickVal);
							}
						}
					}
				}
			}
		}
};

// define mouse buttons - this will need to be enhanced to support multiple modes
Button midButton(ENCODER_SWITCH, "middle button", MOUSE_MIDDLE);
Button leftButton(LEFT_SWITCH, "left button", MOUSE_LEFT);
Button rightButton(RIGHT_SWITCH, "right button", MOUSE_RIGHT);
Button thumbButton(THUMB_SWITCH, "thumb button", THUMB_KEY, false);		// sends keyboard modifier

//------------------------------------- encoder -------------------------------
// Create rotary encoder
Rotary rotary = Rotary(ENCODER_CLOCK, ENCODER_DATA);

int encoderCount = 0;
bool stateChanged = false;

// rotate is called anytime the rotary inputs change state.
void rotate() {
	unsigned char result = rotary.process();
	if (result == DIR_CW) {
		encoderCount++;
		stateChanged = true;
		if (debug_mode)
			Serial.println(encoderCount);
	} else if (result == DIR_CCW) {
		encoderCount--;
		stateChanged = true;
		if (debug_mode)
			Serial.println(encoderCount);
	}
}

void checkEncoder() {
	if (stateChanged) {
		if (debug_mode) {
			Serial.print("encoder count = ");
			Serial.println(encoderCount);
		}
		if (setupMode) {
			// switch to next mode
			nextMode();
		} else {
			switch (mode) {
				case mouse:
					Mouse.move(0, 0, encoderCount);
					break;
				case arrows:
					if (encoderCount > 0) {
						Keyboard.write(KEY_LEFT_ARROW);
						if (debug_mode)
							Serial.println("LEFT ARROW");
					} else {
						Keyboard.write(KEY_RIGHT_ARROW);
						if (debug_mode)
							Serial.println("RIGHT ARROW");
					}
					break;
			}
		}
		encoderCount = 0;
		stateChanged = false;
	}
}

//------------------------------------- support functions ---------------------
void updateButtons() {
	leftButton.Update();
	rightButton.Update();
	midButton.Update();
	thumbButton.Update();
}

void toggleSetup() {
	if (!setupMode) {
		if (leftButton.State() == LOW && rightButton.State() == LOW && midButton.State() == LOW) {
			while (midButton.State() == LOW)	// wait for middle button to be released or we'll switch states again
				midButton.Update();
			setupMode = true;
			Mouse.release(MOUSE_LEFT);		// release the buttons so computer isn't locked up
			Mouse.release(MOUSE_RIGHT);
			Mouse.release(MOUSE_MIDDLE);
			Serial.println("Setup Mode On");
			print_filename();
		}
	} else {
		if (midButton.State() == LOW) {
			setupMode = false;
			Serial.println("Setup Mode Off");
		}
	}
}

// print last part of source file path
void print_filename() {
	char *sptr = strrchr(__FILE__, '/');
	Serial.println(++sptr);
}

//------------------------------------- start ---------------------------------
void setup() {
  void calibrateJoystick();

	attachInterrupt(digitalPinToInterrupt(ENCODER_CLOCK), rotate, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_DATA), rotate, CHANGE);
	Mouse.begin();
	Keyboard.begin();
	Serial.begin(115200);

	// enable use of built-in LED
	pinMode(LED_BUILTIN, OUTPUT);

	// configure pin for external LED
	pinMode(EXTERN_LED, OUTPUT);
	// and turn it off (testing)
	externLed(HIGH);

	if (digitalRead(THUMB_SWITCH) == LOW) {
		debug_mode = true;
	}

	blink(4, LED_PULSETIME);

  // get center values for joystick (assuming you don't push on it at powerup)
  calibrateJoystick();

	// enable debug?
	if (debug_mode) {
		print_filename(); 		// identify source file
		Serial.println("Debug mode enabled");
	}

	while (!Serial) {
		yield();
	}
	Serial.print("Commit Hash: ");
	Serial.println(GIT_COMMIT_HASH);
	Serial.print("Build Date: ");
	Serial.println(BUILD_DATE);
	Serial.println(SOURCE_FILE_NAME);	// print source file name
	Serial.println();
}

//-----------------------------------------------------------------------------
// blink LED without stopping processing
bool running = false;
unsigned long endTime;
int blinkCount = 0;
int blinkDuration = 0;
bool ledOn = false;

void blink(int count, int blinkTime) {
	// Serial.print("bink: count = "); Serial.print(count); Serial.print(" blinkTime = "); Serial.println(blinkTime);
	if (!running) {							// start new cycle, if already running, too bad, can't stack them up
		running = true;
		blinkCount = count + 1;					// remember pulses and their length
		blinkDuration = blinkTime;
		endTime = millis() + blinkDuration;	// set the timer
		// digitalWrite(LED_BUILTIN, HIGH);	// and turn on the LED
		externLed(HIGH);
		ledOn = true;
	}
}

void doBlink() {
	if (running) {
		if (endTime < millis()) {				// has delay elapsed?
			#if defined(DEBUG_BLINK)
			if (debug_mode) {
				Serial.print("ledOn = ");
				Serial.println(ledOn);
			}
			#endif
			if (ledOn) {						// toggle LED
				#if defined(DEBUG_BLINK)
				if (debug_mode) {
					Serial.print("blinkCount = ");
					Serial.print(blinkCount);
					Serial.println(" turning off LED");
				}
				#endif
				// digitalWrite(LED_BUILTIN, LOW);	// turn off
				externLed(HIGH);
				ledOn = false;
				blinkCount--;					// decrement count every time cycle completes
				if (blinkCount > 0) {			// if more to do
					endTime = millis() + blinkDuration;		// restart timer
				} else if (blinkCount == 0) {				// finished, delay a bit before allowing turn on again
					endTime = millis() + DEADTIME;	// long delay after last one cycle
					ledOn = true;				// say it's on so we hit the off code again
				} else {						// we're done
					running = false;
				}
			} else {
				#if defined(DEBUG_BLINK)
				if (debug_mode) {
					Serial.print("blinkCount = ");
					Serial.print(blinkCount);
					Serial.println(" turning on LED");
				}
				#endif
				// digitalWrite(LED_BUILTIN, HIGH);
				externLed(LOW);
				ledOn = true;
				endTime = millis() + blinkDuration;		// restart timer
			}
		}
	}
}

// external LED
void externLed(int state) {
	digitalWrite(EXTERN_LED, state);
}


//------------------------------------- mouse code -----------------------------
// joystick
int range = 12;               // output range x or y movement
int responseDelay = 15;        // response delay of mouse in ms
int threshhold = range / 6;   // resting threshhold
// int threshhold = range / 4;   // resting threshhold


int lastMouseMove = millis();
typedef struct joyAxis {
  int pin;
  int center;
};

joyAxis x = { A3, 0 };
joyAxis y = { A2, 0 };
int xCenter = 0;                      // center values read at boot
int yCenter = 0;

int readAxis(struct joyAxis *axis) {
  int reading = analogRead(axis->pin);

  // remove offset
  reading = reading - axis->center;

  // map reading
  int distance = map(reading, -511, 511, range, -range);
  if (debug_mode){
    Serial.print("Distance: ");
    Serial.println(distance);
  }

  // if the output reading is outside the rest position threshhold, use it
  // int distance = reading - center;
  // int distance = reading;

  if (abs(distance) < threshhold) {
    distance = 0;
  }

  if (debug_mode) {
    Serial.print("Distance: ");
    Serial.println(distance);
  }

  return distance;
}

// read and store joystick center position values

void calibrateJoystick() {
  x.center = analogRead(x.pin);
  y.center = analogRead(y.pin);
}

void checkJoy() {
  int newTime = millis();

  if ((newTime - lastMouseMove) < responseDelay)    // don't move too fast
    return;

  if (debug_mode) {             // display raw data for each axis
    int value = analogRead(x.pin);
    Serial.print("X: ");
    value -= x.center;
    Serial.print(value);
    Serial.print(" ");
    Serial.println(x.center);
    value = analogRead(y.pin);
    Serial.print("Y: ");
    value -= y.center;
    Serial.print(value);
    Serial.print(" ");
    Serial.println(y.center);
  }
  
  lastMouseMove = newTime;

  int xReading = readAxis(&x);
  int yReading = readAxis(&y);

  // move the "mouse" if joystick is away from center position
  if ((xReading != 0) || (yReading != 0)) {
    if (!debug_mode) {    // play it safe - retain control of mouse
    	Mouse.move(xReading, yReading, 0);
	}
  }
}

//------------------------------------- main loop -----------------------------
void loop() {
	if (setupMode) {
		// display current mode (blink extern LED)
		displayMode();
	}
	checkEncoder();
	updateButtons();
	checkJoy();
	toggleSetup();		// go to setup mode?
	doBlink();			// keep the LED running
}
