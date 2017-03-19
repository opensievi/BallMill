// OpenSievi BallMill controller
// https://github.com/opensievi/ballmill
//
// Copyright (c) 2017 Tapio Salonsaari <take@nerd.fi>
// 
// This arudino project is to control ball mill motor. Based
// quite heavily on our thermostat project.
//
// See README.md and LICENSE.txt for more info
//

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Wire.h>

#include "PinChangeInt/PinChangeInt.h"

// Software version and writing time
#define SWVERSION "0.9"
#define SWDATE "03-17"

// Behaviour
#define TIMERS 4 // Number of timers
#define BUTTONS 4 // Number of buttons

// Pin layout
#define RELAY_PIN 12
#define MOSFET_PIN A7

#define STARTBTN_PIN 6
#define STOPBTN_PIN 5
#define ROTARYBTN_PIN 4
#define ROTARYFWD_PIN 2
#define ROTARYRWD_PIN 3
#define POT_PIN A3

// LCD used
#define LCD_WIDTH 16
#define LCD_HEIGHT 2
#define LCD_ADDRESS 0x27

// Init subsystems
LiquidCrystal_I2C lcd(LCD_ADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

int textState = 0;

int cur_phase=0; // current working phase
int cur_running=0; // Run status to track direction change and timer override
int cur_rotation=0; // Which way we're turning
int cur_acceleration=0; // How fast we're turning
int change_speed=0; // track potentiometer changes during run
char phasename[6];
char phasetxt[6];

long timers[TIMERS]; // Countdown timers
long timer_millis; // Internal timer

int timecalc; // Used to calculate spent time

// Default values
unsigned long changeDirection = 300000; // 300000ms = 5min
// 100ms per accel. step * 127 = 12700ms = 12.7 seconds to full speed
unsigned long accelerationDelay = 150; 

// Button handlers
int pot_position=0;
int btn_queue[4];
int rotary_count=0;

// Interrupt function to track down buttons
void trigger_button() {

	int cur_button = PCintPort::arduinoPin;

	switch (cur_button) {
		
		case STARTBTN_PIN:
			btn_queue[0]++;
			break;

		case STOPBTN_PIN:
			btn_queue[1]++;
			break;

		case ROTARYBTN_PIN:
			btn_queue[2]++;
			break;

		default:
			// We don't have this button?
			break;
	}

}

// Interrupt trigger for rotary button, we only
// track the last direction it was turned
void rotary_trigger() {

	int pinA = digitalRead(ROTARYFWD_PIN);
	int pinB = digitalRead(ROTARYRWD_PIN);

	if(pinA != pinB) {
		rotary_count=-1;
	} else {
		rotary_count=1;
	}
}


// Do timer calculations
// This is somewhat hairy since we have incrementing and decreasing
// timers and they're calculated only based on current phases
void countTimers() {

	unsigned long diffMillis = millis() - timer_millis;
	timer_millis = millis();

	for(int i=0;i < TIMERS; i++) {
		
		if(i != 3) {
			if(i == 0) {
				if(cur_running > 0) {
					timers[i] -= diffMillis;
				}
			} else {
				timers[i] -= diffMillis;
			}
		} else {
			// Only calc time when we're running
			if(cur_running == 2) {
				timers[i] += diffMillis;
			}
		}
		if(timers[i] < 0) {
			timers[i] = 0;
		}
	}

	// Do time tracking calculations. This isn't millisecond
	// accurate, but should be close enough for this purpose.
	timecalc += diffMillis;
	if(timecalc >= 1000) {
		timecalc -= 1000;
	}
	
	return;
}	

// Manage current phase changes, see comments for details
void changePhase() {
	
	switch(cur_phase) {
		case 0: // Standby, we do nothing

			// Reset variables
			cur_rotation = 0;
			cur_acceleration = 0;
			sprintf(phasename,"STDBY");
			sprintf(phasetxt," ");

			break;

		case 1: // Acceleration

			sprintf(phasename,"RUN");
			sprintf(phasetxt,"ACCEL");
			//sprintf(phasetxt,"%d", cur_acceleration);

			if(timers[0] == 0 && cur_running != 2) {
				cur_phase = 3;
				cur_running = 0;
			}

			if(timers[2] == 0) {
				cur_acceleration++;
				timers[2] = accelerationDelay;
				analogWrite(MOSFET_PIN, cur_acceleration);
			} 
			
			int upto;
			upto = int(((float)pot_position/100)*255);

			if(cur_acceleration >= upto) {
				// Acceleration finished, move to
				// stable running
				timers[1] = changeDirection;
				cur_phase = 2;
			}

			break;
		case 2: // Running

			if(cur_rotation == 0) {
				sprintf(phasetxt,">>>>");
			} else {
				sprintf(phasetxt,"<<<<");
			}
			// Change rotation direction
			if(timers[1] == 0) {
				cur_phase = 3;
				if(cur_rotation == 0) {
					cur_rotation = 1;
				} else {
					cur_rotation = 0;
				}
			}

			// Timer finished
			if(timers[0] == 0 && cur_running != 2) {
				cur_phase = 3;
				cur_running = 0;
			}

			if(cur_running == 0) {
				cur_phase = 3;
			}

			break;
		case 3: // Deceleration

			int dest;
			dest = 0;
			if(change_speed) {
				dest = int(((float)pot_position/100)*127);
			}

			//sprintf(phasetxt,"%d", cur_acceleration);
			sprintf(phasetxt,"DECEL");
			if(timers[2] == 0) {
				cur_acceleration--;
				timers[2] = accelerationDelay;
				analogWrite(MOSFET_PIN, cur_acceleration);
			} 
			
			if(cur_running == 0) {
				sprintf(phasename,"STOP");
			}
				
			if(cur_acceleration <= dest) {
				// Decelration finished
				if(cur_running > 0) {
					timers[1] = changeDirection;
					change_speed = 0;
					cur_phase = 1;
					if(cur_rotation) {
						digitalWrite(RELAY_PIN, LOW);
					} else {
						digitalWrite(RELAY_PIN, HIGH);
					}
				} else {
					// If we stopped
					digitalWrite(RELAY_PIN, HIGH);
					cur_phase = 0;
				}
			}
		
			break;
		default:
			// Something goes BADLY wrong if this
			// ever happens. We go to infinite loop
			// and shut everything down.
			char buf[20];
			digitalWrite(RELAY_PIN,HIGH); // Shut off burner
			while(1) {
				lcd.home();
				lcd.print("PHASE ERROR            ");
				lcd.setCursor(0,1);
				sprintf(buf,"Phase: %d              ",cur_phase);
				lcd.print(buf);
				delay(10000);
			}
			break;
	}
}

// Function: Setup
//
// Initialize subsystems and start main loop
//
void setup() {
	char version[20];	
	Serial.begin(9600);

	// Output pins
	pinMode(RELAY_PIN, OUTPUT);

	// Input pins
	pinMode(STARTBTN_PIN, INPUT_PULLUP);
	pinMode(STOPBTN_PIN, INPUT_PULLUP);
	pinMode(ROTARYBTN_PIN, INPUT_PULLUP);
	pinMode(ROTARYFWD_PIN, INPUT_PULLUP);
	pinMode(ROTARYRWD_PIN, INPUT_PULLUP);
	pinMode(POT_PIN, INPUT);

	// Initialize interrupts
	attachPinChangeInterrupt(STARTBTN_PIN, trigger_button, FALLING);
	attachPinChangeInterrupt(STOPBTN_PIN, trigger_button, FALLING);
	attachPinChangeInterrupt(ROTARYBTN_PIN, trigger_button, FALLING);
	attachPinChangeInterrupt(ROTARYFWD_PIN, rotary_trigger, CHANGE);

	// Initialize timers
	for(int i=0;i < TIMERS;i++) {
		timers[i] = 0;
	}

	// Initialize button counters
	for(int i=0; i < BUTTONS; i++) {
		btn_queue[i] = 0;
	}

	// Our relay pulls when RELAY_PIN is LOW, this is somewhat
	// inconvinient, but it should work out just fine
	digitalWrite(RELAY_PIN, HIGH);

	lcd.begin(LCD_WIDTH, LCD_HEIGHT); // initialize the lcd 
	lcd.home(); // go home

	// Output some version information on startup
	lcd.print("O-S BallMill");  
	lcd.setCursor ( 0, 1 );        // go to the next line
	sprintf(version, "ver %s %s", SWVERSION, SWDATE);
	lcd.print (version);
	delay ( 1500 );

	lcd.clear();
	return;
}

// Main loop
void loop() {
	char buf[20];
	char buf2[20];
	// If your pot works wrong way change 100, 0 to 0, 100
	// or, change + and - wires on the potentiomenter
	int tmp_pot = map(analogRead(POT_PIN), 0, 1023, 100, 0); 
	if(cur_phase != 0) {
		if(tmp_pot > pot_position) {
			change_speed = 1;
			cur_phase = 1; // accelerate smoothly
		} 
		if(tmp_pot < pot_position) {
			change_speed = 1;
			cur_phase = 3; // decelerate smoothly
		}
	}
	pot_position = tmp_pot;


	// Add / subtract a minute from timer when rotary switch is turned
	if(rotary_count == 1) {
		timers[0]+=300000;
		//timers[0]+=60000;
		rotary_count = 0;
	} 

	if(rotary_count == -1) {
		timers[0]-=300000;
		//timers[0]-=60000;
		if(timers[0] < 0) {
			timers[0] = 0;
		}
		rotary_count = 0;
	}

	// If stop button is pressed
	if(btn_queue[1] > 0) {
		timers[0]=0;
		btn_queue[1]=0;
		if(cur_running > 0) {
			cur_phase = 3;
			cur_running = 0;
		}
	}

	// If start button is pressed
	if(btn_queue[0] > 0) {
		btn_queue[0] = 0;
		if(cur_phase == 0) {
			timers[2] = 0;
			timers[3] = 0;
			cur_acceleration = 0;
			if(timers[0] == 0) {
				cur_running = 2;
			} else {
				cur_running = 1;
			}
			cur_phase = 1;
		}
	}

	countTimers(); // Manage time calculations
	changePhase(); // Manage phases

	int time_calc[2];
	int tmp_time;

	if(cur_running == 2) {
		tmp_time = timers[3] / 1000;
	} else {
		tmp_time = timers[0] / 1000;
	}

	time_calc[0] = tmp_time / 60 /60;
	tmp_time -= time_calc[0]*60*60;
	time_calc[1] = tmp_time / 60;
	tmp_time -= time_calc[1]*60;
	time_calc[2] = tmp_time;

	if(cur_running == 2) {
		sprintf(buf2, "Run : %02dh%02dm%02ds", time_calc[0], time_calc[1], time_calc[2]);
	} else {
		sprintf(buf2, "Time: %02dh%02dm%02ds", time_calc[0], time_calc[1], time_calc[2]);
	}

	sprintf(buf, "%-5s %3d%% %5s", phasename, pot_position, phasetxt);

	lcd.setCursor(0,0);
	lcd.print(buf);
	lcd.setCursor(0,1);
	lcd.print(buf2);

}
