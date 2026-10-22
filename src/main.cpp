#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <HX711.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// ESP32 klasik: DOIT DEVKIT V1 / ESP-WROOM-32.
constexpr uint8_t RFID_SS = 23;
constexpr uint8_t RFID_RST = 18;
constexpr uint8_t RFID_SCK = 22;
constexpr uint8_t RFID_MISO = 19;
constexpr uint8_t RFID_MOSI = 21;
constexpr uint8_t HX_DOUT = 4;
constexpr uint8_t HX_SCK = 15;
constexpr uint8_t LED_PIN = 25;
constexpr uint8_t BUZZER_PIN = 26;
constexpr uint8_t BUZZER_CHANNEL = 0;
constexpr uint32_t INDICATOR_MS = 300;
constexpr unsigned int BUZZER_HZ = 2000;  // Nada untuk buzzer pasif melalui driver.
constexpr char DEVICE_ID[] = "TROLLEY_SENSOR_01";

constexpr uint8_t WINDOW_SIZE = 10;
constexpr float STABLE_BAND_G = 5.0f;
constexpr float CHANGE_THRESHOLD_G = 20.0f;
constexpr uint32_t STABLE_HOLD_MS = 700;
constexpr uint32_t TAG_TIMEOUT_MS = 10000;
constexpr uint32_t HX_TIMEOUT_MS = 1500;
constexpr uint32_t RFID_POLL_MS = 50;
constexpr uint32_t RFID_COOLDOWN_MS = 1200;
constexpr uint32_t LOG_INTERVAL_MS = 1000;
constexpr uint32_t TX_INTERVAL_MS = 5000;

MFRC522 rfid(RFID_SS, RFID_RST);
HX711 scale;
Preferences preferences;

long rawWindow[WINDOW_SIZE] = {};
uint8_t sampleCount = 0;
uint8_t writeIndex = 0;
double rawMean = 0.0;
double rawRange = 0.0;
double rawOffset = 0.0;
float scaleFactor = 0.0f;  // Counts per gram; 0 berarti belum kalibrasi.
float weightG = 0.0f;
float baselineG = 0.0f;
bool calibrated = false;
bool tared = false;
bool hxOnline = false;
bool hxFaultReported = false;
bool hxSaturated = false;
bool rfidOnline = false;
bool preferencesReady = false;
bool stable = false;
bool stabilityTimerRunning = false;
bool baselineValid = false;

uint32_t lastHxMs = 0;
uint32_t stableSinceMs = 0;
uint32_t lastRfidPollMs = 0;
uint32_t lastUidMs = 0;
uint32_t lastLogMs = 0;
uint32_t lastTxMs = 0;
uint32_t sequenceNumber = 0;
char lastUid[21] = {};

struct PendingScan {
  bool active = false;
  char uid[21] = {};
  float referenceG = 0.0f;
  uint32_t scannedAtMs = 0;
};
PendingScan pending;
bool indicatorActive = false;
bool buzzerReady = false;
uint32_t indicatorStartedMs = 0;

void signalItemAdded() {
  indicatorStartedMs = millis();
  indicatorActive = true;
  digitalWrite(LED_PIN, HIGH);
  if (buzzerReady) ledcWrite(BUZZER_CHANNEL, 128); // Duty 50% pada resolusi 8 bit.
}

void serviceIndicators() {
  if (indicatorActive &&
      static_cast<uint32_t>(millis() - indicatorStartedMs) >= INDICATOR_MS) {
    digitalWrite(LED_PIN, LOW);
    if (buzzerReady) ledcWrite(BUZZER_CHANNEL, 0);
    indicatorActive = false;
  }
}

// Deklarasi eksplisit agar default argument juga aman saat disalin ke .ino.
void sendToServerPlaceholder(const char *event, const char *uid = nullptr,
                             bool hasDelta = false, float deltaG = 0.0f);

bool weightAvailable() {
  return calibrated && tared && hxOnline &&
         sampleCount == WINDOW_SIZE && isfinite(weightG);
}

void resetWindow() {
  sampleCount = 0;
  writeIndex = 0;
  rawMean = 0.0;
  rawRange = 0.0;
  weightG = 0.0f;
  stable = false;
  stabilityTimerRunning = false;
  baselineValid = false;
}

// Boundary TX. Fungsi ini HANYA mencetak: tidak mengirim ke ESP lain.
void sendToServerPlaceholder(const char *event, const char *uid,
                             bool hasDelta, float deltaG) {
  Serial.println("[TX-SIMULASI] Data sudah ditransfer ke ESP utama "
                 "(SIMULASI; hanya Serial Monitor).");
  Serial.printf("{\"device_id\":\"%s\",\"seq\":%lu,\"uptime_ms\":%lu,"
                "\"event\":\"%s\",\"uid\":",
                DEVICE_ID, static_cast<unsigned long>(++sequenceNumber),
                static_cast<unsigned long>(millis()), event);
  if (uid != nullptr && uid[0] != '\0') {
    Serial.printf("\"%s\"", uid);  // UID berasal dari format hex internal.
  } else {
    Serial.print("null");
  }
  Serial.print(",\"weight_g\":");
  if (weightAvailable()) Serial.print(weightG, 1);
  else Serial.print("null");
  Serial.print(",\"delta_g\":");
  if (hasDelta && weightAvailable()) Serial.print(deltaG, 1);
  else Serial.print("null");
  Serial.printf(",\"stable\":%s,\"calibrated\":%s,\"tared\":%s,"
                "\"hx711_online\":%s,\"simulated\":true,"
                "\"delivered\":false,\"transport\":\"serial_placeholder\"}\n",
                stable ? "true" : "false", calibrated ? "true" : "false",
                tared ? "true" : "false", hxOnline ? "true" : "false");
}

void serviceHx711() {
  const uint32_t now = millis();
  if (!scale.is_ready()) {
    if (static_cast<uint32_t>(now - lastHxMs) > HX_TIMEOUT_MS &&
        !hxFaultReported) {
      hxFaultReported = true;
      hxOnline = false;
      tared = false;  // Data sesudah putus sensor perlu nol ulang.
      resetWindow();
      sendToServerPlaceholder("HX711_TIMEOUT", pending.active ? pending.uid : nullptr);
      pending.active = false;
      Serial.println("[HX711] Tidak ada sampel baru. Cek kabel; "
                     "setelah pulih, kosongkan platform dan TARE ulang.");
    }
    return;
  }

  // Satu sampel per kesempatan. Tidak menunggu read_average(10).
  const long raw = scale.read();
  lastHxMs = now;
  if (!hxOnline) Serial.println("[HX711] Sampel diterima.");
  hxOnline = true;
  hxFaultReported = false;
  if (raw <= -8388608L || raw >= 8388607L) {
    tared = false;
    resetWindow();
    if (!hxSaturated) {
      sendToServerPlaceholder("HX711_SATURATION", pending.active ? pending.uid : nullptr);
      Serial.println("[HX711] SATURASI. Cek wiring/beban; setelah pulih, TARE ulang.");
    }
    hxSaturated = true;
    pending.active = false;
    return;
  }
  hxSaturated = false;
  rawWindow[writeIndex] = raw;
  writeIndex = (writeIndex + 1) % WINDOW_SIZE;
  if (sampleCount < WINDOW_SIZE) ++sampleCount;

  double sum = 0.0;
  long minimum = rawWindow[0];
  long maximum = rawWindow[0];
  for (uint8_t i = 0; i < sampleCount; ++i) {
    sum += rawWindow[i];
    if (rawWindow[i] < minimum) minimum = rawWindow[i];
    if (rawWindow[i] > maximum) maximum = rawWindow[i];
  }
  rawMean = sum / sampleCount;
  rawRange = static_cast<double>(maximum) - static_cast<double>(minimum);
  if (calibrated && tared) {
    weightG = static_cast<float>((rawMean - rawOffset) / scaleFactor);
  }

  const bool quietWindow = weightAvailable() &&
                          rawRange / fabs(scaleFactor) <= STABLE_BAND_G;
  if (!quietWindow) {
    stable = false;
    stabilityTimerRunning = false;
  } else {
    if (!stabilityTimerRunning) {
      stabilityTimerRunning = true;
      stableSinceMs = now;
    }
    stable = static_cast<uint32_t>(now - stableSinceMs) >= STABLE_HOLD_MS;
  }
}

void serviceWeightEvents() {
  if (pending.active &&
      static_cast<uint32_t>(millis() - pending.scannedAtMs) >= TAG_TIMEOUT_MS) {
    sendToServerPlaceholder("TAG_TIMEOUT", pending.uid);
    pending.active = false;
    Serial.println("[RFID] Waktu habis; tidak ada perubahan berat stabil yang cocok.");
  }
  if (!weightAvailable() || !stable) return;
  if (!baselineValid) {
    baselineG = weightG;
    baselineValid = true;
    Serial.println("[SISTEM] Berat acuan siap. Scan satu tag, lalu ubah beban.");
    return;
  }

  const float reference = pending.active ? pending.referenceG : baselineG;
  const float delta = weightG - reference;
  if (fabs(delta) < CHANGE_THRESHOLD_G) return;

  if (pending.active) {
    if (delta > 0) {
      signalItemAdded();
      Serial.printf("\n*** [ITEM_ADDED] UID=%s | bertambah=%.3f kg | total=%.3f kg ***\n",
                    pending.uid, delta / 1000.0, weightG / 1000.0);
    }
    sendToServerPlaceholder(delta > 0 ? "ITEM_ADDED" : "ITEM_REMOVED",
                            pending.uid, true, delta);
    pending.active = false;
  } else {
    sendToServerPlaceholder("WEIGHT_CHANGED_UNPAIRED", nullptr, true, delta);
  }
  baselineG = weightG;  // Perubahan ini hanya dilaporkan sekali.
}

void serviceRfid() {
  const uint32_t now = millis();
  if (!rfidOnline ||
      static_cast<uint32_t>(now - lastRfidPollMs) < RFID_POLL_MS) return;
  lastRfidPollMs = now;
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  char uid[21] = {};
  const uint8_t uidSize = rfid.uid.size;
  if (uidSize > 0 && uidSize <= 10) {
    for (uint8_t i = 0; i < uidSize; ++i) {
      snprintf(uid + 2 * i, sizeof(uid) - 2 * i, "%02X",
               static_cast<unsigned int>(rfid.uid.uidByte[i]));
    }
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  if (uid[0] == '\0') return;
  if (strcmp(uid, lastUid) == 0 &&
      static_cast<uint32_t>(now - lastUidMs) < RFID_COOLDOWN_MS) return;
  strcpy(lastUid, uid);
  lastUidMs = now;
  Serial.printf("\n*** [RFID TERBACA] UID=%s ***\n", uid);

  if (pending.active) {
    sendToServerPlaceholder("RFID_BUSY", uid);
    Serial.println("[RFID] Selesaikan scan sebelumnya; tag baru tidak menggantinya.");
    return;
  }
  if (!weightAvailable() || !stable || !baselineValid) {
    sendToServerPlaceholder("RFID_SCAN_UNVERIFIED", uid);
    Serial.println("[RFID] UID terbaca. Tunggu kalibrasi, TARE, dan berat stabil; "
                   "angkat tag lalu scan ulang.");
    return;
  }
  strcpy(pending.uid, uid);
  pending.referenceG = weightG;
  pending.scannedAtMs = now;
  pending.active = true;
  sendToServerPlaceholder("RFID_SCAN", uid);
  Serial.println("[RFID] Masukkan/keluarkan barang dalam 10 detik; satu barang saja.");
}

void printHelp() {
  Serial.println("HELP | STATUS | TARE | CAL <berat_gram>");
  Serial.println("RX PING | RX GET_STATUS | RX TARE");
  Serial.println("Akhiri perintah dengan Newline. TARE hanya saat platform kosong.");
  Serial.println("Tampilan berat dalam kg; CAL dan field JSON weight_g/delta_g dalam gram.");
  Serial.println("Uji sementara iPhone 13 biasa tanpa aksesori: TARE kosong, taruh HP, CAL 174.");
  Serial.println("174 g adalah massa nominal; gunakan beban terukur untuk kalibrasi akurat.");
  Serial.println("Faktor disimpan jika NVS berhasil. Setelah restart cukup TARE saat kosong.");
}

void handleCommand(char *command, bool fromServer) {
  if (strcmp(command, "STATUS") == 0 || strcmp(command, "GET_STATUS") == 0) {
    sendToServerPlaceholder("STATUS");
  } else if (strcmp(command, "PING") == 0) {
    sendToServerPlaceholder("PONG");
  } else if (strcmp(command, "HELP") == 0 && !fromServer) {
    printHelp();
  } else if (strcmp(command, "TARE") == 0) {
    if (pending.active || !hxOnline || sampleCount != WINDOW_SIZE ||
        (calibrated && rawRange / fabs(scaleFactor) > STABLE_BAND_G)) {
      Serial.println("\n*** [TARE DITOLAK] Tunggu sampel penuh, berat diam, "
                     "dan scan selesai. ***");
      return;
    }
    rawOffset = rawMean;
    tared = true;
    resetWindow();
    Serial.printf("\n*** [TARE OK] Offset=%.0f counts. Tunggu pembacaan stabil lagi. ***\n", rawOffset);
    sendToServerPlaceholder("TARE_OK");
  } else if (strncmp(command, "CAL ", 4) == 0 && !fromServer) {
    char *end = nullptr;
    const float knownMassG = strtof(command + 4, &end);
    if (end == command + 4 || *end != '\0' || !isfinite(knownMassG) ||
        knownMassG <= 0 || !hxOnline || sampleCount != WINDOW_SIZE ||
        !tared || pending.active) {
      Serial.println("\n*** [CAL DITOLAK] TARE saat kosong, taruh beban diketahui, "
                     "tunggu diam, lalu ketik contoh: CAL 174 ***");
      return;
    }
    const float candidate = static_cast<float>((rawMean - rawOffset) / knownMassG);
    if (fabs(rawMean - rawOffset) <= fmax(10.0, rawRange * 10.0) ||
        !isfinite(candidate) || fabs(candidate) < 0.000001f ||
        rawRange / fabs(candidate) > STABLE_BAND_G) {
      Serial.println("\n*** [CAL DITOLAK] Beban belum stabil atau selisih raw tidak valid. ***");
      return;
    }
    scaleFactor = candidate;  // Nilai negatif juga sah, sesuai arah sinyal.
    calibrated = true;
    bool saved = false;
    if (preferencesReady) {
      saved = preferences.putFloat("factor", scaleFactor) == sizeof(float);
    }
    resetWindow();
    Serial.printf("\n*** [CAL OK] Faktor=%.6f counts/g; tersimpan=%s ***\n",
                  scaleFactor, saved ? "ya" : "tidak (RAM saja)");
    sendToServerPlaceholder("CALIBRATION_OK");
  } else {
    Serial.println("[CMD] Perintah tidak dikenal. Ketik HELP.");
  }
}

// Boundary RX. Untuk sekarang sumbernya keyboard Serial Monitor.
void receiveFromServerPlaceholder(char *command) {
  Serial.printf("[RX-SIMULASI] Perintah ESP utama: %s (input Serial Monitor).\n",
                command);
  handleCommand(command, true);
}

void serviceSerial() {
  static char buffer[64] = {};
  static uint8_t length = 0;
  static bool discardLine = false;
  uint8_t budget = 32;  // Input panjang tidak mengambil seluruh waktu loop.
  while (Serial.available() && budget > 0) {
    --budget;
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (discardLine) {
        Serial.println("[CMD] Baris terlalu panjang; seluruh baris dibuang.");
      } else if (length > 0) {
        buffer[length] = '\0';
        char *command = buffer;
        while (*command && isspace(static_cast<unsigned char>(*command))) ++command;
        char *end = command + strlen(command);
        while (end > command && isspace(static_cast<unsigned char>(end[-1]))) --end;
        *end = '\0';
        if (strncmp(command, "RX ", 3) == 0) {
          receiveFromServerPlaceholder(command + 3);
        } else if (*command != '\0') {
          handleCommand(command, false);
        }
      }
      length = 0;
      discardLine = false;
    } else if (!discardLine) {
      if (length < sizeof(buffer) - 1) buffer[length++] = c;
      else discardLine = true;
    }
  }
}

void servicePeriodicOutput() {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastLogMs) >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    if (sampleCount > 0) {
      Serial.printf("[SENSOR] raw=%.0f, sampel=%u/%u, ", rawMean,
                    static_cast<unsigned int>(sampleCount),
                    static_cast<unsigned int>(WINDOW_SIZE));
    } else {
      Serial.print("[SENSOR] menunggu sampel, ");
    }
    if (weightAvailable()) {
      Serial.printf("berat=%.3f kg, noise_pp=%.3f kg, ",
                    weightG / 1000.0, rawRange / fabs(scaleFactor) / 1000.0);
    }
    else Serial.print("berat=belum valid, ");
    Serial.printf("stabil=%s, scan_pending=%s\n", stable ? "ya" : "tidak",
                  pending.active ? "ya" : "tidak");
  }
  if (static_cast<uint32_t>(now - lastTxMs) >= TX_INTERVAL_MS) {
    lastTxMs = now;
    sendToServerPlaceholder("TELEMETRY");
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.begin(115200);
  // Core ESP32 2.x: timer harus siap sebelum ledcAttachPin membaca duty channel.
  buzzerReady = ledcSetup(BUZZER_CHANNEL, BUZZER_HZ, 8) > 0;
  if (buzzerReady) {
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    ledcWrite(BUZZER_CHANNEL, 0);
  } else {
    Serial.println("[BUZZER] Inisialisasi PWM gagal; indikator LED tetap aktif.");
  }
  delay(300);
  Serial.println("\nSMART MARKET - RC522 + HX711 - satu ESP32");
  Serial.println("Seluruh TX/RX server pada versi ini adalah SIMULASI.");
  preferencesReady = preferences.begin("trolley_sensor", false);
  if (preferencesReady) scaleFactor = preferences.getFloat("factor", 0.0f);
  calibrated = isfinite(scaleFactor) && fabs(scaleFactor) >= 0.000001f;
  Serial.printf("Kalibrasi tersimpan: %s. TARE tetap wajib setelah boot.\n",
                calibrated ? "ada" : "belum ada");

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  rfid.PCD_Init();
  delay(4);
  const byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  rfidOnline = version != 0x00 && version != 0xFF;
  Serial.printf("RC522 VersionReg=0x%02X; komunikasi awal=%s\n", version,
                rfidOnline ? "terdeteksi" : "gagal, cek wiring");
  scale.begin(HX_DOUT, HX_SCK);  // Channel A, gain 128.
  lastHxMs = millis();
  printHelp();
}

void loop() {
  serviceIndicators();
  serviceHx711();
  serviceWeightEvents();  // Laporkan perubahan lama sebelum menerima scan baru.
  serviceSerial();
  serviceRfid();
  servicePeriodicOutput();
  delay(2);
}
