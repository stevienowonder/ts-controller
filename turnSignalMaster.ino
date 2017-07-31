/*
Turn Signal Controller code for FFR AC Cobra

Version: 0.9
Date: 31 July, 2017
Author: Steven Todd
*/

boolean debug = false;

class SwitchIO {

	int inPin;
	int outPin;
	int swState;
	int lastSwState;

	boolean debouncePending;
	boolean outputOn;

	unsigned long debounceTimerStart;

	const unsigned long debounceInterval = 20;

public:
	enum Mode {
		LATCHED = 0,
		MOMENTARY = 1
	} mode;

	SwitchIO(const int inPinAttach, const int outPinAttach, enum SwitchIO::Mode swMode) : inPin(inPinAttach), outPin(outPinAttach), mode(swMode) {
	}

	void setup() {
		pinMode(inPin, INPUT_PULLUP);
		pinMode(outPin, OUTPUT);
		digitalWrite(outPin, LOW);
		outputOn = false;
		debouncePending = false;
		swState = LOW;
		lastSwState = LOW;
	}

	void turnOn() {
		digitalWrite(outPin, HIGH);
		outputOn = true;
	}

	void turnOff() {
		digitalWrite(outPin, LOW);
		outputOn = false;
	}

	void toggle() {
		digitalWrite(outPin, !digitalRead(outPin));
		outputOn = !outputOn;
	}

	void update() {
		lastSwState = swState;
		swState = digitalRead(inPin);

		switch (mode) {
		case MOMENTARY:
			if (swState > lastSwState) {															// trigger on positive edge only
				debounceTimerStart = millis();
				debouncePending = true;
			}
			else if ((debouncePending) && ((unsigned long)(millis() - debounceTimerStart) > debounceInterval)) {
				toggle();																								// toggle output state after debounce
				debouncePending = false;
			}
			break;																										
		case LATCHED:
			if (swState != lastSwState) {															// trigger on any transition
				debounceTimerStart = millis();
				debouncePending = true;
			}
			else if (debouncePending && ((unsigned long)(millis() - debounceTimerStart) > debounceInterval)) {
				if (swState == HIGH) {																	// turn on after debounce
					this->turnOn();
				}
				else if (swState == LOW) {															// turn off after debounce
					this->turnOff();
				}
				debouncePending = false;
			}
			break;
		}
	}

	int status() {
		return outputOn;
	}
};

class TurnSignalOutput {

	int pin;
	int flashState = 1;

	unsigned long timeout;
	unsigned long startTime = 0;
	unsigned long lastFlashTime = 0;

	const unsigned long flashInterval = 1000;

	enum State {
		OFF = 0,
		ON = 1
	};

	State state = OFF;

public:
	TurnSignalOutput(int attachTo) : pin(attachTo) {
	}

	void setup() {
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}

	void turnOn(unsigned long duration) {
		timeout = duration;
		digitalWrite(pin, HIGH);
		startTime = millis();
		lastFlashTime = startTime;
		state = ON;
	}

	unsigned int turnOff(unsigned long delay) {
		if (timeout == 0) {																					// turn off now
			digitalWrite(pin, LOW);
			state = OFF;
			startTime = 0;
			lastFlashTime = 0;
		}
		else {
			timeout = delay;																					// schedule timer variable for delayed off function
			startTime = millis();																			// let update() trap for delay interval
		}
		return(timeout);
	}

	void flash() {
		digitalWrite(pin, !digitalRead(pin));
		lastFlashTime = millis();
	}

	void update() {
		if ((unsigned long)(millis() - startTime) >= timeout) {
			this->turnOff(0);																					// reached timeout threshold, turn off now
		}
		else if ((unsigned long)(millis() - lastFlashTime) >= flashInterval) {
			this->flash();																						// reached flash interval
		}
	}

	int status() {
		return state;
	}
};

class SteeringCancel {

	int inputVal;
	int noiseVal;
	int inputState;
	int prevInputState;

	int inPinIR;
	int outPinIR;
	const int error = -1;

	const int delayRead = 50;																			// micro sec to wait after turning off led before taking noise reading
	const int digLowThresh = 160;																	// analog values below this = digital low state (0 - 1000 sensor range)
	const int digHighThresh = 400;																// analog values above this = digital high state (0 - 1000 sensor range)

	const unsigned long timeout = 3000;														// Timeout for no movement in PENDING_RETURN state

	unsigned long currTime;
	unsigned long timeoutStart;
	
	public:
	SteeringCancel(int inPinAttach, int outPinAttach) : inPinIR(inPinAttach), outPinIR(outPinAttach) {
	}

	enum CancelStatus {
		INACTIVE = 0,
		PENDING_MOVEMENT = 1,
		PENDING_RETURN = 2,
		PENDING_CANCEL = 3,
		CANCEL = 4
	} cancelStatus;

	void setup() {
		cancelStatus = INACTIVE;
		pinMode(outPinIR, OUTPUT);
	}

	void activate() {
		cancelStatus = PENDING_MOVEMENT;
		inputState = this->readIR();																// set baseline for switch position
		prevInputState = inputState;
	}

	void deactivate() {
		cancelStatus = INACTIVE;
	}

	int readIR() {
		digitalWrite(outPinIR, HIGH);																// turn ON LED
		delayMicroseconds(250);																			// delay before read
		inputVal = analogRead(inPinIR);															// take reading from phototransistor :noise+signal
		digitalWrite(outPinIR, LOW);																// turn OFF LED
		delayMicroseconds(250);																			// delay before read
		inputVal -= analogRead(inPinIR);														// subtract noise from signal

		if (inputVal < digLowThresh) {
			inputVal = 0;																							// low logic level
		}
		else if (inputVal > digHighThresh) {
			inputVal = 1;																							// high logic level
		}
		else {
			inputVal = error;																					// indeterminate state
		}
		return inputVal;
	}

	int update() {
		prevInputState = inputState;
		if ((inputState = this->readIR()) == error) {								
			inputState = prevInputState;															// no change if read is indeterminate
		}

		switch (cancelStatus) {
		case PENDING_MOVEMENT:
			if (inputState > prevInputState) {												// trigger on positive edge
				cancelStatus = PENDING_RETURN;													// initial steering movement detected
			}
			break;
		case PENDING_RETURN:
			if (inputState > prevInputState) {												// trigger on positive edge
				cancelStatus = PENDING_CANCEL;													// return steering movement detected
				timeoutStart = millis();																// set timer start to watch for additional movement 
			}
			break;
		case PENDING_CANCEL:
			if (inputState > prevInputState) {												// trigger on positive edge
				timeoutStart = millis();																// reset timer start if additional movement detected 
			}
			else if ((unsigned long)(millis() - timeoutStart) > timeout) {
				cancelStatus = CANCEL;																	// no steering input within before timeout; cancel 
			}
			break;
		}
	}

	int status() {
		return cancelStatus;
	}
};

// main program

const int swPin = 2;																						// digital pin connected to switch output
const int xPin = 0;																							// analog pin connected to X output
const int yPin = 1;																							// analog pin connected to Y output

const int irInPin = 2;																					// analog pin for IR cancel sensor 
const int irOutPin = 6;																					// digital pin for IR sensor LED

const int rTurnOutPin = 7;																			// digital pin connected to right turn signal out
const int lTurnOutPin = 4;																			// digital pin connected to left turn signal out

const int revLimInPin = 10;
const int revLimOutPin = LED_BUILTIN;

int leverPos = 500;																							// initially set to neutral lever position
int lTurnReqPos = 700;
int rTurnReqPos = 300;

unsigned long currTime;
unsigned long debounceTimerStart;

const unsigned long debounceInterval = 50;
const unsigned long latchInterval = 300;
const unsigned long timeOnUnlatched = 6000;
const unsigned long timeOnLatched = 90000;
const unsigned long now = 0;
const unsigned long steeringCancelDelay = 3000;

enum State {
	OFF = 0,
	RIGHT_PENDING = 1,
	LEFT_PENDING = 2,
	RIGHT = 3,
	LEFT = 4,
	RIGHT_LATCHED = 5,
	LEFT_LATCHED = 6,
	RIGHT_CANCELED = 7,
	LEFT_CANCELED = 8
};

State state = OFF;

TurnSignalOutput rTurnSignalOutput(rTurnOutPin);
TurnSignalOutput lTurnSignalOutput(lTurnOutPin);
SteeringCancel steeringCancel(irInPin, irOutPin);
SwitchIO revLimiter(revLimInPin, revLimOutPin, SwitchIO::Mode::MOMENTARY);

void setup() {

	if (debug) {
		Serial.begin(9600);
	}

	pinMode(swPin, INPUT);
	pinMode(xPin, INPUT);
	pinMode(yPin, INPUT);

	rTurnSignalOutput.setup();
	lTurnSignalOutput.setup();
	steeringCancel.setup();
	revLimiter.setup();
}

void loop() {

	leverPos = analogRead(xPin);
	currTime = millis();

	rTurnSignalOutput.update();																	// update class object status
	lTurnSignalOutput.update();
	steeringCancel.update();
	revLimiter.update();

	if (leverPos < rTurnReqPos) {																// right switch input detected
		switch (state) {
		case RIGHT_CANCELED:																			// switch active after cancel, wait for switch off
			break;
		case OFF:																									// initial loop w/switch input detected
			debounceTimerStart = currTime;
			state = RIGHT_PENDING;
			break;
		case RIGHT_PENDING:
			if ((unsigned long)(currTime - debounceTimerStart) > debounceInterval) {
				if (rTurnSignalOutput.status()) {
					rTurnSignalOutput.turnOff(now);
					state = RIGHT_CANCELED;															// reselect detected; cancel turn signal
				}
				else {
					state = RIGHT;																			// new right turn signal request detected
					rTurnSignalOutput.turnOn(timeOnUnlatched);
					steeringCancel.activate();													// start steering cancel watchdog process
					lTurnSignalOutput.turnOff(now);
				}
			}
			break;
		case RIGHT:
			if ((unsigned long)(currTime - debounceTimerStart) > latchInterval) {
				state = RIGHT_LATCHED;																// right TS active and latched on
				rTurnSignalOutput.turnOn(timeOnLatched);
			}
			break;
		case LEFT_PENDING:																				// initial loop with direction change detected
			debounceTimerStart = currTime;
			state = RIGHT_PENDING;
			lTurnSignalOutput.turnOff(now);
			break;
		case LEFT:																								// initial loop with direction change detected
			debounceTimerStart = currTime;
			state = RIGHT_PENDING;
			lTurnSignalOutput.turnOff(now);
			break;
		case LEFT_LATCHED:																				// initial loop with direction change detected
			debounceTimerStart = currTime;
			state = RIGHT_PENDING;
			lTurnSignalOutput.turnOff(now);
			break;
		case LEFT_CANCELED:																				// initial loop with direction change detected
			debounceTimerStart = currTime;
			state = RIGHT_PENDING;
			break;
		}
	}
	else if (leverPos > lTurnReqPos) {													// left switch input detected
		switch (state) {
		case LEFT_CANCELED:																				// switch active after cancel, wait for switch off
			break;
		case OFF:																									// initial loop w/switch input detected
			debounceTimerStart = currTime;
			state = LEFT_PENDING;
			break;
		case LEFT_PENDING:
			if ((unsigned long)(currTime - debounceTimerStart) > debounceInterval) {
				if (lTurnSignalOutput.status()) {
					lTurnSignalOutput.turnOff(now);
					state = LEFT_CANCELED;															// relselect detected; cancel turn signal
				}
				else {
					state = LEFT;																				// new turn signal request detected
					lTurnSignalOutput.turnOn(timeOnUnlatched);
					steeringCancel.activate();													// start steering cancel watchdog timer
					rTurnSignalOutput.turnOff(now);
				}
			}
			break;
		case LEFT:
			if ((unsigned long)(currTime - debounceTimerStart) > latchInterval) {
				state = LEFT_LATCHED;																	// left TS active and latched on
				lTurnSignalOutput.turnOn(timeOnLatched);
			}
			break;
		case RIGHT_PENDING:																				// initial loop with direction change detected
			debounceTimerStart = currTime;
			state = LEFT_PENDING;
			rTurnSignalOutput.turnOff(now);
			break;
		case RIGHT:																								// initial loop with direction change detected
			debounceTimerStart = currTime;
			state = LEFT_PENDING;
			rTurnSignalOutput.turnOff(now);
			break;
		case RIGHT_LATCHED:																				// initial loop with direction change detected
			debounceTimerStart = currTime;
			state = LEFT_PENDING;
			rTurnSignalOutput.turnOff(now);
			break;
		case RIGHT_CANCELED:																			// initial loop with direction change detected
			debounceTimerStart = currTime;
			state = LEFT_PENDING;
			rTurnSignalOutput.turnOff(now);
			break;
		}
	}
	else {																											// no turn signal input request active
		state = OFF;
	}

	if (rTurnSignalOutput.status()) {														// check steering cancel watchdog if output active
		if (steeringCancel.status() == SteeringCancel::CancelStatus::CANCEL) {
			rTurnSignalOutput.turnOff(steeringCancelDelay);
			steeringCancel.deactivate();
		}
	}
	if (lTurnSignalOutput.status()) {
		if (steeringCancel.status() == SteeringCancel::CancelStatus::CANCEL) {
			lTurnSignalOutput.turnOff(steeringCancelDelay);
			steeringCancel.deactivate();
		}
	}

	if (debug) {
		Serial.print("Lever pos: ");
		Serial.print(leverPos);
		Serial.print("\n");

		Serial.print("State: ");
		Serial.print(state);
		Serial.print("\n");

		Serial.print("Current Time: ");
		Serial.print(currTime);
		Serial.print("\n");

		Serial.print("Debounce Start: ");
		Serial.print(debounceTimerStart);
		Serial.print("\n");

		Serial.print("Debounce Interval: ");
		Serial.print((unsigned long)(currTime - debounceTimerStart));
		Serial.print("\n");

		Serial.print("R Output State: ");
		Serial.print(rTurnSignalOutput.status());
		Serial.print("\n");

		Serial.print("L Output State: ");
		Serial.print(lTurnSignalOutput.status());
		Serial.print("\n");

		Serial.print("Steering state: ");
		Serial.print(steeringCancel.status());
		Serial.print("\n\n");
	}
}

