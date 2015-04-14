#include "IRremote.h"
#include "EEPROM.h"
#include "EEPROManything.h"
#include "MotorPot.h"
#include "Helper.h"

// Pins
#define IR_REC 2	// bind IR receiver out to pin 2

#define ENC_A 3		// bind 2-bit switch pin A to pin 3
#define ENC_B 4		// bind 2-bit switch pin B to pin 4

#define INPUT_1 8	// bind input 1 relay to pin 8
#define INPUT_2 9	// bind input 2 relay to pin 9
#define INPUT_3 14	// bind input 3 relay to pin 14 (A0)
#define INPUT_4 15	// bind input 4 relay to pin 15 (A1)

#define LED_1 10	// bind LED 1 to pin 10
#define LED_2 11	// bind LED 2 to pin 11
#define LED_3 12	// bind LED 3 to pin 12
#define LED_4 13	// bind LED 4 to pin 13

#define MUTE 16		// bind the mute relay to pin 16 (A2)

#define MOTOR_1	18	// bind H-bridge leg 1 (pin 2, 1A) to pin 18 (A4)              
#define MOTOR_2	17	// bind H-bridge leg 2 (pin 7, 2A) to pin 17 (A3)
#define ENABLE 19	// bind H-bridge enable pin to pin 19 (A5)

// Hardware configuration
#define NUMBER_OF_INPUTS 4	// this sets the amount of used inputs (max 4 inputs)
 
// Navigation & Control 
#define INCREASE 0x01
#define DECREASE 0x02

// Other flags
#define FIRST_USE 0x40

// IRremote class stuff
IRrecv irrecv(IR_REC);			// bind IR receiver OUT the pin that corresponds to IR_REC
decode_results results;			// Holds all the IR values
unsigned long IRcode;			// for temporary storing the receives IR codes
unsigned long lastKey;			// for storing the last valid IR code that is not a repeat code

// MotorPot instantiation
MotorPot mpot(MOTOR_1, MOTOR_2, ENABLE);

// Variables
unsigned long inputSwitchMillis = 350;	// for recording the time after each input switching activity
unsigned long muteMillis = 350;			// for recording the time after each mute/unmute activity
unsigned long remoteMillis = 0;			// for recording the time after IR remote activity
unsigned long blinkMillis = 500;		

bool muted = true;						// for indicating that the amp is muted
bool overrideInputSelector = false;		// for indicating that the input selector switch is being overridden by the remote control
bool ledOn = false;						// for indicating that the LED is ON (=true) or OFF (=false)

byte selectedInput;						// holds the current selected input
byte msb;								// for storing the ENC_A pin state (most significant bit);
byte lsb; 								// for storing the ENC_B pin state (least significant bit);

// Constants
const long startUpDelayTime = 3000;			// delay time in milliseconds at start up before unmute
const long minimalRotatingTime = 200;		// the minimal time that the motor pot keeps rotating after when the volume is changed
const long minimalInputSwitchTime = 350;	// minimal time in milliseconds between input switching activity
const long minimalRemoteTime = 200;			// minimal time between each key press of the remote
const long minimalBlinkTime = 500;			// minimal time between the blinking of the LED


// SETUP: MAIN ENTRY POINT OF THE PROGRAM
void setup()
{	
	//Serial.begin(9600);	// For debugging purposes

	irrecv.enableIRIn();	// Start the IR receiver
				
	setPinModes();			// set pin modes for all pins that are not set by other functions
	setPinStates();			// sets pin modes for all pins that are not set by other functions

	mpot.begin();			// initialize the motor control 	
	mpot.enable_a();		// enable motor 1
	
	setInput();				// select the input by reading the input selector switch	
	delayStartUp(startUpDelayTime);	// wait for the power supply to stabilize before unmuting
	unmuteAmp();			// unmute the DCB1	

	// reset counters	
	remoteMillis = millis();
	inputSwitchMillis = millis();
} // END SETUP


// MAIN LOOP
void loop()
{
	// if the IR receiver starts receiving
	if (irrecv.decode(&results)) 
	{				
		setAppleIRCode();	// set the IR code variable		
		irrecv.resume();	// Receive the next value		
	}
	
	// if a remote key is pressed
	if (IRcode != 0)
	{		
		if (millis()-remoteMillis >= minimalRemoteTime)
		{
			handleRemote();	// handle the received IR code					
			remoteMillis = millis();
		}
	}
		
	// if the input was manually selected with the input selector switch
	if (millis() - inputSwitchMillis >= minimalInputSwitchTime)
	{	
		if (inputChanged())
		{			
			setInput();
		}		
		inputSwitchMillis = millis();
	}	
	
	// if the motorpot is rotating then stop it from further rotating if the 
	// minimal rotating time is exceeded
	if ((mpot.rotating_a_cw() || mpot.rotating_a_ccw()) 
		&& millis() - remoteMillis >= minimalRotatingTime)
	{
		mpot.stop_a();	// stop rotating the motor pot 		
	}
	
	// blink the LED that corresponds to the selected input
	// to indicate the mute state
	if (muted)
	{
		BlinkInputLed();		
	}
} // END OF MAIN LOOP

// delays the startup by the given time in millis
void delayStartUp(const int startUpDelayTime)
{
	long timeToStop = millis() + startUpDelayTime;
	while (millis() < timeToStop)
	{
		BlinkInputLed();		
	}
}

// Sets the pin modes of all ATmega328 pins that are
// not set by other functions
void setPinModes()
{
	pinMode(INPUT_1, OUTPUT);
	pinMode(INPUT_2, OUTPUT);
	pinMode(INPUT_3, OUTPUT);
	pinMode(INPUT_4, OUTPUT);
	pinMode(MUTE, OUTPUT);
	pinMode(ENC_A, INPUT);
	pinMode(ENC_B, INPUT);
	pinMode(LED_1, OUTPUT);
	pinMode(LED_2, OUTPUT);
	pinMode(LED_3, OUTPUT);
	pinMode(LED_4, OUTPUT);
}

// Sets the pin states of all ATmega328 pins that are
// not set by other functions
void setPinStates()           
{
	digitalWrite(ENC_A, HIGH);		// raise internal pull up
	digitalWrite(ENC_B, HIGH);		// raise internal pull up
}

// Checks if the input is changed manually
bool inputChanged()
{
	return digitalRead(ENC_A) != msb || digitalRead(ENC_B) != lsb;
}

void setInput()
{		
	msb = digitalRead(ENC_A);
	lsb = digitalRead(ENC_B);
	
	byte encoded = (msb << 1) | lsb; //converting the 2 pin value to single number
	
	deactivateSelectedInput();	// first close the currently selected selected input (break)
	
	switch (encoded)
	{
		case 0b00:
		selectedInput = 1;
		break;
		case 0b01:
		selectedInput = 2;
		break;
		case 0b10:
		selectedInput = 3;
		break;
		case 0b11:
		selectedInput = 4;
		break;
		default:
		selectedInput = 2;
		break;
	}
	delay(20);
	activateSelectedInput();
}

// sets the IR code variable
void setAppleIRCode()
{
	if (results.value == REPEAT)
	{
		IRcode = lastKey;	// make IRcode the same as the last valid IR code
	}
	else
	{
		if (results.value >> 16 == PRE_DATA) // code received from a Apple remote
		{
			IRcode = (results.value >> 8) & 0xff;
			IRcode = (IRcode << 1) & 0xff;
		} 
		else // code received from another remote, so set to 0 to abort any further operation
		{
			IRcode = lastKey = 0;			
		}	
	}
}

// handles the received IR code
void handleRemote()
{
	switch (IRcode)
	{
		case KEY_UP:
		increaseVolume();			// increase the volume by rotating the potentiometer cw
		lastKey = KEY_UP;	
		break;
		case KEY_DOWN:				// increase the volume by rotating the potentiometer ccw
		decreaseVolume();
		lastKey = KEY_DOWN;	
		break;
		case KEY_LEFT:
		switchInput(DECREASE);		// decrease the selected input
		lastKey = 0;
		break;
		case KEY_RIGHT:
		switchInput(INCREASE);		// increase the selected input	
		lastKey = 0;
		break;
		case KEY_CENTER:
		// do nothing
		lastKey = 0;
		break;
		case KEY_MENU:
		// do nothing
		lastKey = 0;
		break;
		case KEY_PLAY_PAUSE:		
		if (muted)
		{
			unmuteAmp();			// unmute the pre-amp
		}
		else
		{
			muteAmp();				// mute the pre-amp
		}
		lastKey = 0;
		break;
		default:
		// do nothing, because the code could possible be received from another remote control 
		break;
	}	
	IRcode = 0;					// reset the IR code, so the function doesn't get called unwanted	
}

// increases the volume by rotating the motor pot clock wise
void increaseVolume()
{
	if (!mpot.rotating_a_cw())
	{
		mpot.rotate_a_cw(); // start rotating clock wise		
	}			
}

// decreases the volume by rotating the motor pot counter clock wise
void decreaseVolume()
{
	if (!mpot.rotating_a_ccw())
	{
		mpot.rotate_a_ccw(); // start rotating counter clock wise	
	}
}

// activates the selected input and turns the corresponding LED On
void activateSelectedInput()
{
	if (!muted)
	{
		switch (selectedInput)
		{
			case 1:
			digitalWrite(INPUT_1, HIGH);
			digitalWrite(LED_1, (ledOn = true));
			break;
			case 2:
			digitalWrite(INPUT_2, HIGH);
			digitalWrite(LED_2, (ledOn = true));
			break;
			case 3:
			digitalWrite(INPUT_3, HIGH);
			digitalWrite(LED_3, (ledOn = true));
			break;
			case 4:
			digitalWrite(INPUT_4, HIGH);
			digitalWrite(LED_4, (ledOn = true));
			break;
		}		
	}	
}

// switches to the next or previous input (use break before make method)
// if input 4 is selected and the user tries to increase the input than 
// it rolls over back to input 1
void switchInput(byte value)
{		
	deactivateSelectedInput();	// first close the currently selected selected input (break)
	delay(20);					// wait 20 milli seconds to be sure there the connection has been broken
	Helper::SetPointerValue(value, &selectedInput, NUMBER_OF_INPUTS, 1); // set the variable
	activateSelectedInput();
	inputSwitchMillis = millis();	// record the timestamp at which the input was switched
}

// deactivates the currently selected input
void deactivateSelectedInput()
{
	switch (selectedInput)
	{
		case 1:
		digitalWrite(INPUT_1, LOW);
		digitalWrite(LED_1, LOW);
		break;
		case 2:
		digitalWrite(INPUT_2, LOW);
		digitalWrite(LED_2, LOW);
		break;
		case 3:
		digitalWrite(INPUT_3, LOW);
		digitalWrite(LED_3, LOW);
		break;
		case 4:
		digitalWrite(INPUT_4, LOW);
		digitalWrite(LED_4, LOW);
		break;
	}
}

// mutes the DCB1
void muteAmp()
{	
	deactivateSelectedInput();	// break the input, so no signal is reaching the B1
	muted = true;				// set muted flag to true
}

// unmute's the DCB1
void unmuteAmp()
{	
	muted = false;				// set muted flag to false	
	activateSelectedInput();	// connect the selected input	
	digitalWrite(MUTE, HIGH);	// make sure the output is not shorted to ground
}

// turns the LED On or Off if blinkMillis are equal or larger then the minimal blink time 
void BlinkInputLed()
{
	if (millis() - blinkMillis >= minimalBlinkTime)
	{
		switch (selectedInput)
		{
			case 1:
			digitalWrite(LED_1, (ledOn = !ledOn));
			break;
			case 2:
			digitalWrite(LED_2, (ledOn = !ledOn));
			break;
			case 3:
			digitalWrite(LED_3, (ledOn = !ledOn));
			break;
			case 4:
			digitalWrite(LED_4, (ledOn = !ledOn));
			break;
		}	
		blinkMillis = millis();	
	}
}

