#ifndef MYSERIAL_H
#define MYSERIAL_H

#include <HardwareSerial.h>

// Serial debug
#define SERIAL_DEBUG	0
#define SERIAL_SPEED	115200 

void init_serial() {
	#if SERIAL_DEBUG
	Serial.begin(SERIAL_SPEED);
	#endif
}

template <class T>
void print(const T msg) {
	#if SERIAL_DEBUG
	Serial.print(msg);
	#endif
}

void printbyte(const char c) {
	#if SERIAL_DEBUG
	Serial.write(c);
	#endif
}

template <class T>
void println(const T msg) {
	#if SERIAL_DEBUG
	Serial.println(msg);
	#endif
}

#endif
