#include "../../src/main.cpp"
#include <iostream>
#include <limits>

void command(const std::string &text) {
  for (char c : text + "\n") Serial.input.push_back(c);
  while (Serial.available()) serviceSerial();
}
void samples(long raw, int n = 25) {
  for (int i = 0; i < n; ++i) {
    nowMs += 100;
    scale.readings.push_back(raw);
    serviceHx711();
    serviceWeightEvents();
  }
}
void scan(byte value, byte length = 4) {
  nowMs += RFID_POLL_MS;
  rfid.uid.size = length;
  for (byte i = 0; i < length; ++i) rfid.uid.uidByte[i] = value + i;
  rfid.present = true;
  serviceRfid();
}
bool event(const char *name) {
  return Serial.output.find(std::string("\"event\":\"") + name + "\"") != std::string::npos;
}
void calibrate() {
  samples(200000);
  command("TARE");
  assert(tared);
  samples(144320); // 174 g with a negative factor of -320 counts/g.
  command("CAL 174");
  assert(calibrated && fabs(scaleFactor + 320) < 0.001);
  assert(preferences.stored == scaleFactor);
  samples(200000);
  command("TARE");
  samples(200000);
  assert(weightAvailable() && stable && baselineValid);
}

int main(int argc, char **argv) {
  assert(argc == 2);
  const std::string scenario = argv[1];
  if (scenario == "restart") preferences.stored = -320;
  if (scenario == "rfid_offline") rfid.version = 0xFF;
  if (scenario == "buzzer_failure") ledcSetupOk = false;
  setup();
  if (scenario == "startup") {
    assert(!weightAvailable());
    command("TARE");
    command("CAL 174");
    assert(!tared && !calibrated);
    scan(1, 10);
    assert(event("RFID_SCAN_UNVERIFIED") && !pending.active);
    assert(Serial.output.find("0102030405060708090A") != std::string::npos);
    assert(toneCalls == 0 && pinLevels[25] == LOW);
  } else if (scenario == "restart") {
    assert(calibrated && !tared && !weightAvailable());
    samples(200000);
    command("TARE");
    samples(144320);
    assert(weightAvailable() && fabs(weightG - 174) < 0.01);
  } else if (scenario == "rfid_offline") {
    assert(!rfidOnline);
    calibrate();
    scan(1);
    assert(!pending.active);
  } else {
    calibrate();
    Serial.output.clear();
    if (scenario == "transactions") {
      scan(1);
      assert(pending.active && event("RFID_SCAN"));
      samples(144320);
      assert(event("ITEM_ADDED") && !pending.active);
      Serial.output.clear();
      samples(144320);
      assert(!event("ITEM_ADDED") && !event("WEIGHT_CHANGED_UNPAIRED"));
      scan(1);
      samples(200000);
      assert(event("ITEM_REMOVED") && !pending.active);
      assert(toneCalls == 1); // Only ITEM_ADDED sounds, not removal.
    } else if (scenario == "unpaired") {
      samples(196800); // 10 g is below the 20 g threshold.
      assert(!event("WEIGHT_CHANGED_UNPAIRED"));
      samples(144320);
      assert(event("WEIGHT_CHANGED_UNPAIRED"));
      assert(toneCalls == 0);
    } else if (scenario == "busy_timeout") {
      scan(1);
      scan(1);
      assert(!event("RFID_BUSY"));
      scan(2);
      assert(event("RFID_BUSY") && std::string(pending.uid) == "01020304");
      command("TARE");
      command("CAL 174");
      assert(pending.active && !event("TARE_OK") && !event("CALIBRATION_OK"));
      samples(200000, 105);
      assert(event("TAG_TIMEOUT") && !pending.active);
      assert(toneCalls == 0);
    } else if (scenario == "timeout_recovery") {
      scan(1);
      nowMs += HX_TIMEOUT_MS + 1;
      serviceHx711();
      assert(event("HX711_TIMEOUT") && !tared && !pending.active && !weightAvailable());
      samples(200000);
      assert(!weightAvailable());
      command("TARE");
      samples(200000);
      assert(weightAvailable() && stable);
    } else if (scenario == "commands") {
      for (const char *bad : {"CAL 0", "CAL -1", "CAL nan", "CAL inf", "CAL 174x", "CAL "}) command(bad);
      assert(!event("CALIBRATION_OK") && scaleFactor == -320);
      command(std::string(70, 'X') + "TARE");
      assert(!event("TARE_OK"));
      command("  RX PING  \r");
      command("RX GET_STATUS");
      assert(event("PONG") && event("STATUS"));
      command("RX TARE");
      assert(event("TARE_OK"));
    } else if (scenario == "kg_noise") {
      samples(144320);
      servicePeriodicOutput();
      assert(Serial.output.find("berat=0.174 kg") != std::string::npos);
      for (int i = 0; i < 20; ++i) samples(i % 2 ? 140000 : 150000, 1);
      assert(!stable);
      scan(2);
      assert(event("RFID_SCAN_UNVERIFIED"));
    } else if (scenario == "saturation") {
      scan(1);
      samples(8388607, 1);
      assert(!weightAvailable() && !tared && !pending.active);
      command("TARE");
      assert(!tared);
      samples(200000);
      command("TARE");
      samples(200000);
      assert(weightAvailable());
    } else if (scenario == "storage_failure") {
      preferences.writeOk = false;
      samples(144320);
      command("CAL 174");
      assert(calibrated && event("CALIBRATION_OK"));
      assert(Serial.output.find("tidak (RAM saja)") != std::string::npos);
    } else if (scenario == "buzzer_failure") {
      assert(!buzzerReady && !ledcAttached && toneFrequency == 0);
      scan(1);
      samples(144320, 17);
      assert(event("ITEM_ADDED") && pinLevels[25] == HIGH && toneCalls == 0);
      nowMs += 300;
      loop();
      assert(pinLevels[25] == LOW && pinLevels[26] == LOW);
    } else if (scenario == "indicators" || scenario == "indicators_wrap") {
      if (scenario == "indicators_wrap") nowMs = std::numeric_limits<uint32_t>::max() - 1800;
      scan(1);
      assert(toneCalls == 0 && pinLevels[25] == LOW);
      samples(144320, 17);
      assert(event("ITEM_ADDED"));
      assert(pinModes[25] == OUTPUT && pinModes[26] == OUTPUT);
      assert(pinLevels[25] == HIGH && toneFrequency == 2000 && toneCalls == 1);
      const uint32_t started = nowMs;
      command("RX PING");
      assert(event("PONG") && nowMs == started); // Serial remains responsive.
      nowMs = started + 299;
      loop();
      assert(pinLevels[25] == HIGH && toneFrequency == 2000);
      loop(); // millis() is now 301 ms past the start, including across rollover.
      assert(pinLevels[25] == LOW && toneFrequency == 0);
      assert(pinLevels[26] == LOW && toneCalls == 1);
    } else if (scenario == "wraparound") {
      nowMs = std::numeric_limits<uint32_t>::max() - 500;
      samples(200000, 1);
      scan(1);
      samples(144320);
      assert(event("ITEM_ADDED"));
    } else { assert(false); }
  }
  std::cout << Serial.output;
}
