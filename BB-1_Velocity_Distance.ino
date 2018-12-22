/*
* BB-1_Velocity_Distance.ino
*
* Created: 22.12.2018
* Author: Lukas
*
* Based on "Efficiently Reading Quadrature With Interrupts"
* http://makeatronics.blogspot.de/2013/02/efficiently-reading-quadrature-with.html
* http://homediyelectronics.com/projects/arduino/arduinoprogramminghcsr04withinterrupts/
*
* TimerOne library:
* https://github.com/PaulStoffregen/TimerOne
*
*/

#include "Arduino.h"
#include "Wire.h"
#include "TimerOne.h"

#define SLAVE_ADDRESS 0x08

#define HCSR04_TRIGGER_PIN 6	// pin connected to HC-SR04 trigger

#define TRIGGER_TIME 20					// time in microseconds trigger is set to high
#define TRIGGER_TIME_INTERVAL	2000	// maximum trigger time interval in TRIGGER_TIME

//volatile boolean M1_interrupt = false;
//volatile boolean M2_interrupt = false;

volatile int16_t enc_count_M1 = 0;
volatile int16_t enc_count_M2 = 0;

volatile int32_t front_echo_start;		// front echo start in microseconds
volatile int32_t front_echo_end;		// front echo end in microseconds
volatile int32_t front_echo_duration;	// front echo duration in microseconds

volatile int32_t rear_echo_start;		// rear echo start in microseconds
volatile int32_t rear_echo_end;			// rear echo end in microseconds
volatile int32_t rear_echo_duration;	// rear echo duration in microseconds

volatile boolean front_echo = false;
volatile boolean rear_echo = false;
volatile boolean timer = false;

volatile uint16_t trigger_time_count = 0;	// counter for TRIGGER_TIME_INTERVAL
volatile uint8_t state = 1;	// state variable

byte message[4];

// stores which combination of current and previous encoder state lead to an increase or decrease of the encoder count and which are not defined
const int8_t lookup_table[] = {0, 0, 0, 0, 0, -1, 1, 0, 0, 1, -1, 0, 0, 0, 0, 0};

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

void requestEvent() {
	static uint8_t SREG_bak;
	
	SREG_bak = SREG;	//save global interrupt state
	noInterrupts();
	message[0] = (enc_count_M1 >> 8) & 0xFF;
	message[1] = enc_count_M1 & 0xFF;
	SREG = SREG_bak; 	//restore interrupt state
	
	SREG_bak = SREG;	//save global interrupt state
	noInterrupts();
	message[2] = (enc_count_M2 >> 8) & 0xFF;
	message[3] = enc_count_M2 & 0xFF;
	SREG = SREG_bak; 	//restore interrupt state

	enc_count_M1 = 0;
	enc_count_M2 = 0;
	
	Wire.write(message, 4);
}

void encoder_isr_M1() {
	// stores the current and previous state of both encoder channels (A, B)
	static uint8_t enc_state = 0;
	
	// shift the previous encoder state to the left
	enc_state = enc_state << 1;
	// add the current encoder state (PIND stores the values of digital pins 0-7)
	enc_state = (enc_state & 0b1010) | ((PIND & 0b10100) >> 2);
	
	// change encounter count according to encoder state
	enc_count_M1 = enc_count_M1 + lookup_table[enc_state & 0b1111];
	
	//M1_interrupt = true;
}

void encoder_isr_M2() {
	// stores the current and previous state of both encoder channels (A, B)
	static uint8_t enc_state = 0;
	
	// shift the previous encoder state to the left
	enc_state = enc_state << 1;
	// add the current encoder state (PIND stores the values of digital pins 0-7)
	enc_state = (enc_state & 0b1010) | ((PIND & 0b101000) >> 3);
	
	// change encounter count according to encoder state
	enc_count_M2 = enc_count_M2 - lookup_table[enc_state & 0b1111];
	
	//M2_interrupt = true;
}

ISR(PCINT0_vect) {
	switch (digitalRead(8)) {
		case HIGH:	// echo pin change was a rising edge (start of echo pulse)
			rear_echo_start = micros();
			break;
		case LOW:	// echo pin change was a falling edge (end of echo pulse)
			rear_echo_end = micros();
			rear_echo_duration = rear_echo_end - rear_echo_start;	// calculate echo pulse duration
			rear_echo = true;
			break;
	}
}

ISR(PCINT2_vect) {
	switch (digitalRead(7)) {
		case HIGH:	// echo pin change was a rising edge (start of echo pulse)
			front_echo_start = micros();
			break;
		case LOW:	// echo pin change was a falling edge (end of echo pulse)
			front_echo_end = micros();
			front_echo_duration = front_echo_end - front_echo_start;	// calculate echo pulse duration
			front_echo = true;
			break;
	}
}

void timer_isr() {
	if (++trigger_time_count >= TRIGGER_TIME_INTERVAL) {
		trigger_time_count = 0;	// reset trigger_time_count
		state = 1;
	}
	
	switch(state) {
		case 0:		// idle state
			break;
		case 1:		// start trigger pulse
			digitalWrite(HCSR04_TRIGGER_PIN, HIGH);	// set trigger pin to high
			state = 2;
			break;
		case 2:		// finish trigger pulse
			digitalWrite(HCSR04_TRIGGER_PIN, LOW);	// set trigger pin to low
			state = 0;
			timer = true;
			break;
	}
}

void setup() {
	// join i2c bus
	Wire.begin(SLAVE_ADDRESS);
	// register event
	Wire.onRequest(requestEvent);

	#ifdef DEBUG
	// initialize serial communication
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo eNUMeration, others continue immediately
	#endif
	
	pinMode(2, INPUT);
	pinMode(3, INPUT);
	
	attachInterrupt(digitalPinToInterrupt(2),encoder_isr_M2,CHANGE);
	attachInterrupt(digitalPinToInterrupt(3),encoder_isr_M2,CHANGE);
	
	pinMode(HCSR04_TRIGGER_PIN, OUTPUT);	// configure trigger pin as output
	digitalWrite(HCSR04_TRIGGER_PIN, LOW);	// set trigger pin to low
	
	pinMode(7, INPUT);		// configure front echo pin as input
	digitalWrite(7, HIGH);	// set front echo pin to high
	pinMode(8, INPUT);		// configure rear echo pin as input
	digitalWrite(8, HIGH);	// set rear echo pin to high
	
	noInterrupts();
	PCICR |= (1 << PCIE0) | (1 << PCIE2); // interrupts from B and D

	PCMSK0 |= (1 << PB0); // bit 0 on B (pin 8) will interrupt
	PCMSK2 |= (1 << PD7); // bit 7 on D (pin 7) will interrupt
	interrupts();

	Timer1.initialize(TRIGGER_TIME);	// initialize timer 1
	Timer1.attachInterrupt(timer_isr);	// attach interrupt service routine timer_isr to timer 1
}

void loop() {
	/*if (M1_interrupt) {
	DEBUG_PRINTLN(enc_count_M1 % 1000);
	M1_interrupt = false;
	}
	
	if (M2_interrupt) {
	DEBUG_PRINT("\t"); DEBUG_PRINTLN(enc_count_M2 % 1000);
	M2_interrupt = false;
	}*/
	
	static float front_distance = 2000;	// distance to obstacle front in centimeter
	static float rear_distance = 2000;	// distance to obstacle rear in centimeter
	
	if (front_echo) {
		front_echo = false;
		front_distance = front_echo_duration * 0.01716;
	}
	
	if (rear_echo) {
		rear_echo = false;
		rear_distance = rear_echo_duration * 0.01716;
	}
	
	if (timer == true) {
		timer = false;

		if (front_distance < 200) {
			DEBUG_PRINT(front_distance);
		}
		else {
			DEBUG_PRINT("");
		}
		
		DEBUG_PRINT("\t");
		
		if (rear_distance < 200) {
			DEBUG_PRINTLN(rear_distance);
		}
		else {
			DEBUG_PRINTLN("");
		}
	}
}