// Stand-ins for the Arduino/Teensy APIs that Event_Logic.ino calls, so the
// original C++ can be compiled and run on a laptop. Each stub records what the
// flight computer tried to do (fire a pin, write EEPROM, ...) instead of doing it.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

typedef uint8_t byte;
typedef bool boolean;

const uint8_t LOW = 0;
const uint8_t HIGH = 1;

// implemented in harness.cpp: writes one row to the events log
void logAction(const char* kind, const std::string& name, const std::string& detail = "");

void digitalWrite(uint8_t pin, uint8_t value);

struct EEPROMStub {
  void update(int address, uint8_t value) {
    logAction("EEPROM", "update", "address=" + std::to_string(address) + " value=" + std::to_string(value));
  }
};
inline EEPROMStub EEPROM;

struct RadioTimerStub {
  void begin(void (*)(), uint32_t interval) {
    logAction("RADIO_RATE", "radioTimer.begin", "interval_us=" + std::to_string(interval));
  }
};
inline RadioTimerStub radioTimer;

// GPS time is only copied into the touchdown record, so zeros are fine
struct GPSTimeStub {
  uint8_t hour() { return 0; }
  uint8_t minute() { return 0; }
  uint8_t second() { return 0; }
  uint8_t centisecond() { return 0; }
};
struct GPSStub {
  GPSTimeStub time;
};
inline GPSStub GPS;

// Event_Logic.ino only prints in bench-test mode, which the harness doesn't use
struct SerialStub {
  template <typename T> void print(T) {}
  template <typename T> void println(T) {}
  void println() {}
};
inline SerialStub Serial;

inline void syncSD() { logAction("SD", "syncSD"); }
