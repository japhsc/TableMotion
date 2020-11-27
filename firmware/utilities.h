/*
    utilities.h
*/

#ifndef UTILITIES_H
#define UTILITIES_H

#include <avr/eeprom.h>
#include "myserial.h"

template <typename T> 
bool load_eeprom(const unsigned int &addr, T *store) {
  byte* ptr = new byte[sizeof(T)];
  eeprom_read_block((void*) ptr, (void*)addr, sizeof(T));
  
  bool fresh = true;
  for (uint8_t i=0; i<sizeof(T); ++i)
    fresh &= ( *(ptr+i) == 0xFF );
    
  if (fresh) {
    eeprom_write_block((const void*)store, (void*)addr, sizeof(T));
  } else {
    memcpy(store, ptr, sizeof(T));
  }

  delete [] ptr;
  return fresh;
}

template <typename T> 
void save_eeprom(const unsigned int &addr, T *store) {
  eeprom_update_block((const void*)store, (void*)addr, sizeof(T));
}

template <typename T> 
void reset_eeprom(const unsigned int &addr, T stores) {
  byte store[sizeof(T)];
  for (uint8_t i=0; i<sizeof(T); ++i)
    store[i]=0xFF;
  eeprom_update_block((const void*)&store, (void*)addr, sizeof(T));
}

bool read_digital(const uint8_t &pin, bool& value) {
  bool val = digitalRead(pin);
  if (val == value) {
    return false;
  } else {
    value = val;
    return true;
  }
}

bool read_pwm_state(uint8_t pin) {
  return (*portInputRegister(digitalPinToPort(pin)) & digitalPinToBitMask(pin));
}

bool sample_button(const uint8_t &pin, bool &var, unsigned long &ts, unsigned long &dur){
  if (read_digital(pin, var)) {
    if (var) {
      dur = millis()-ts;
      return true;
    } else {
      ts = millis();
    }
  }
  return false;
}

bool sample_action(const uint8_t &pin, bool &var, unsigned long &ts, uint8_t &action){
  unsigned long dur;
  if (sample_button(pin, var, ts, dur)) {
    if (dur > 1000) {
      action = 2;
    } else {
      action = 1;
    }

    print("Button ");
    print(pin);
    print(" \t ");
    print(action);
    print(" \t ");
    println(dur);
    
    return true;
  }
  return false;
}

uint8_t sample_hall( const uint8_t pin_h_1, const uint8_t pin_h_2, 
                          bool &val_h_1, bool &val_h_2, long &steps) {
  bool upd_h_1 = read_digital(pin_h_1, val_h_1);
  bool upd_h_2 = read_digital(pin_h_2, val_h_2);
  if (upd_h_1 ^ upd_h_2) {
    if (val_h_1 && val_h_2) {
      if (upd_h_2) {
        steps++;
      } else {
        steps--;
      }
      return 2;
    }
    return 1;
  }
  return 0;
}

void blinkn(const uint8_t pin, const uint8_t n, int t = 200) {
  for (uint8_t i=0; i<n; ++i) {
    digitalWrite(pin, HIGH);
    delay(t);
    digitalWrite(pin, LOW);
    delay(t);
  }
}

#endif
