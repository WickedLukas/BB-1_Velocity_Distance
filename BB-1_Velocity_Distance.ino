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
#define TRIGGER_TIME_INTERVAL	1500	// maximum trigger time interval in TRIGGER_TIME

//volatile boolean enc_M1_interrupt = false;
//volatile boolean enc_M2_interrupt = false;

volatile int16_t enc_count_M1 = 0;	// motor 1 encoder count
volatile int16_t enc_count_M2 = 0;	// motor 2 encoder count

volatile boolean front_meas = false;	// stores if a new measurement is available for front HC-SR04
volatile boolean rear_meas = false;		// stores if a new measurement is available for rear HC-SR04

volatile uint16_t front_echo_duration;	// front echo duration in microseconds
volatile uint16_t rear_echo_duration;	// rear echo duration in microseconds

volatile uint8_t front_distance = 255;	// distance to obstacle front in centimeter
volatile uint8_t rear_distance = 255;	// distance to obstacle rear in centimeter

// stores which combination of current and previous encoder state lead to an increase or decrease of the encoder count and which are not defined
const int8_t lookup_table[] = {0, 0, 0, 0, 0, -1, 1, 0, 0, 1, -1, 0, 0, 0, 0, 0};

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).
//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

void requestEvent() {
	static byte message[6];
	
	message[0] = (enc_count_M1 >> 8) & 0xFF;	// most significant bits for motor 1 encoder count
	message[1] = enc_count_M1 & 0xFF;			// least significant bits for motor 1 encoder count
	message[2] = (enc_count_M2 >> 8) & 0xFF;	// most significant bits for motor 2 encoder count
	message[3] = enc_count_M2 & 0xFF;			// least significant bits for motor 2 encoder count
	message[4] = front_distance;
	message[5] = rear_distance;
	
	Wire.write(message, 6);
	
	// reset encoder count
	enc_count_M1 = 0;
	enc_count_M2 = 0;
}

void enc_isr_M1() {
	// stores the current and previous state of both encoder channels (A, B)
	static uint8_t enc_state = 0;
	
	// shift the previous encoder state to the left
	enc_state = enc_state << 1;
	// add the current encoder state (PIND stores the values of digital pins 0-7)
	enc_state = (enc_state & 0b1010) | ((PIND & 0b10100) >> 2);
	
	// update encoder count according to the encoder state
	enc_count_M1 = enc_count_M1 + lookup_table[enc_state & 0b1111];
	
	//enc_M1_interrupt = true;
}

void enc_isr_M2() {
	// stores the current and previous state of both encoder channels (A, B)
	static uint8_t enc_state = 0;
	
	// shift the previous encoder state to the left
	enc_state = enc_state << 1;
	// add the current encoder state (PIND stores the values of digital pins 0-7)
	enc_state = (enc_state & 0b1010) | ((PIND & 0b101000) >> 3);
	
	// update encoder count according to the encoder state
	enc_count_M2 = enc_count_M2 - lookup_table[enc_state & 0b1111];
	
	//enc_M2_interrupt = true;
}

// front HC-SR04
ISR(PCINT2_vect) {
	static uint32_t echo_start;	// echo start in microseconds
	
	switch (digitalRead(7)) {
		case HIGH:	// echo pin change was a rising edge (start of echo pulse)
			echo_start = micros();
			break;
		case LOW:	// echo pin change was a falling edge (end of echo pulse)
			// calculate front echo pulse duration and constrain it to measure a maximum distance of 255 cm (255 * 58 = 14790)
			front_echo_duration = constrain(micros() - echo_start, 0, 14790);
			front_meas = true;
			break;
	}
}

// rear HC-SR04
ISR(PCINT0_vect) {
	static uint32_t echo_start;	// rear echo start in microseconds
	
	switch (digitalRead(8)) {
		case HIGH:	// echo pin change was a rising edge (start of echo pulse)
			echo_start = micros();
			break;
		case LOW:	// echo pin change was a falling edge (end of echo pulse)
			// calculate rear echo pulse duration and constrain it to measure a maximum distance of 255 cm (255 * 58 = 14790)
			rear_echo_duration = constrain(micros() - echo_start, 0, 14790);
			rear_meas = true;
			break;
	}
}

// generate trigger used for front and rear HC-SR04
void timer_isr() {
	static uint16_t trigger_time_count = 0;	// counter for TRIGGER_TIME_INTERVAL
	static uint8_t state = 1;	// state variable
	
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
	
	// prepare encoder pins
	pinMode(2, INPUT);	// interrupt pin for encoder M1
	pinMode(3, INPUT);	// interrupt pin for encoder M2
		
	attachInterrupt(digitalPinToInterrupt(2), enc_isr_M1, CHANGE);	// attach interrupt service routine enc_isr_M1 to encoder M1 interrupt pin
	attachInterrupt(digitalPinToInterrupt(3), enc_isr_M2, CHANGE);	// attach interrupt service routine enc_isr_M2 to encoder M2 interrupt pin
	
	// prepare HC-SR04 pins
	pinMode(HCSR04_TRIGGER_PIN, OUTPUT);	// configure trigger pin as output
	digitalWrite(HCSR04_TRIGGER_PIN, LOW);	// set trigger pin to low
	
	pinMode(7, INPUT);		// configure front echo pin as input
	digitalWrite(7, HIGH);	// set front echo pin to high
	pinMode(8, INPUT);		// configure rear echo pin as input
	digitalWrite(8, HIGH);	// set rear echo pin to high
	
	noInterrupts();
	PCICR |= (1 << PCIE0) | (1 << PCIE2); // interrupts from B and D
	
	PCMSK0 |= (1 << PB0); // bit 0 on B (digital pin 8) will interrupt
	PCMSK2 |= (1 << PD7); // bit 7 on D (digital pin 7) will interrupt
	interrupts();
	
	Timer1.initialize(TRIGGER_TIME);	// initialize timer 1
	Timer1.attachInterrupt(timer_isr);	// attach interrupt service routine timer_isr to timer 1
}

void loop() {
	if (front_meas) {
		front_meas = false;
		// calculate front distance, which is limited to 255, since front_echo_duration is limited to 58 * 255 = 14790
		front_distance = front_echo_duration / 58;
		
		//DEBUG_PRINTLN(front_distance);
	}
	
	if (rear_meas) {
		rear_meas = false;
		// calculate rear distance, which is limited to 255, since rear_echo_duration is limited to 58 * 255 = 14790
		rear_distance = rear_echo_duration / 58;
		
		//DEBUG_PRINT("\t"); DEBUG_PRINTLN(rear_distance);
	}
	
	/*if (enc_M1_interrupt) {
		DEBUG_PRINTLN(enc_count_M1 % 1000);
		enc_M1_interrupt = false;
	}
	
	if (enc_M2_interrupt) {
		DEBUG_PRINT("\t"); DEBUG_PRINTLN(enc_count_M2 % 1000);
		enc_M2_interrupt = false;
	}*/
}