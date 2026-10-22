#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <string>
#include <deque>
#include <cassert>
#include <cstring>
using byte = uint8_t;
static uint32_t nowMs = 0;
uint32_t millis() { return nowMs; }
void delay(uint32_t ms) { nowMs += ms; }
constexpr int OUTPUT = 1, LOW = 0, HIGH = 1;
static int pinLevels[40] = {}, pinModes[40] = {};
static unsigned toneFrequency = 0, toneCalls = 0;
static bool ledcConfigured = false, ledcAttached = false, ledcSetupOk = true;
static unsigned ledcFrequency = 0;
void pinMode(uint8_t pin, int mode) { pinModes[pin] = mode; }
void digitalWrite(uint8_t pin, int level) { pinLevels[pin] = level; }
double ledcSetup(uint8_t channel, double frequency, uint8_t bits) {
  assert(channel == 0 && bits == 8);
  ledcConfigured = ledcSetupOk;
  ledcFrequency = static_cast<unsigned>(frequency);
  return ledcSetupOk ? frequency : 0;
}
void ledcAttachPin(uint8_t pin, uint8_t channel) {
  assert(ledcConfigured && pin == 26 && channel == 0);
  ledcAttached = true;
}
void ledcWrite(uint8_t channel, uint32_t duty) {
  assert(ledcConfigured && ledcAttached && channel == 0);
  assert(duty == 0 || duty == 128);
  toneFrequency = duty ? ledcFrequency : 0;
  if (duty) ++toneCalls;
}

struct SerialMock {
  std::string output;
  std::deque<char> input;
  void begin(int) {}
  int available() { return static_cast<int>(input.size()); }
  int read() { char c = input.front(); input.pop_front(); return c; }
  void print(const char *s) { output += s; }
  void print(double value, int digits) {
    char buffer[128]; snprintf(buffer, sizeof(buffer), "%.*f", digits, value); output += buffer;
  }
  void println(const char *s = "") { print(s); output += '\n'; }
  void printf(const char *format, ...) {
    char buffer[2048]; va_list args; va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args); va_end(args); output += buffer;
  }
} Serial;
struct SPIMock { void begin(int, int, int, int) {} } SPI;

struct HX711 {
  std::deque<long> readings;
  void begin(int, int, int = 128) {}
  bool is_ready() { return !readings.empty(); }
  long read() { assert(is_ready()); long x = readings.front(); readings.pop_front(); return x; }
};

struct MFRC522 {
  enum { VersionReg };
  struct { byte size = 0; byte uidByte[10] = {}; } uid;
  bool present = false;
  byte version = 0x92;
  MFRC522(int, int) {}
  void PCD_Init() {}
  byte PCD_ReadRegister(int) { return version; }
  bool PICC_IsNewCardPresent() { return present; }
  bool PICC_ReadCardSerial() { return present; }
  void PICC_HaltA() { present = false; }
  void PCD_StopCrypto1() {}
};

struct Preferences {
  float stored = 0;
  bool openOk = true, writeOk = true;
  bool begin(const char *, bool) { return openOk; }
  float getFloat(const char *, float fallback) { return stored == 0 ? fallback : stored; }
  size_t putFloat(const char *, float value) {
    if (!writeOk) return 0;
    stored = value; return sizeof(float);
  }
};
