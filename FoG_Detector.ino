#include <Arduino.h>
#include <NimBLEDevice.h>
#include "FogDetector.h"

// ==================================================
// HARDWARE PINS (Right-side breadboard layout)
// ==================================================
// Change to 4 and 5 if using left-side pins D4 and D5
static const int LED_PIN   = 26; // Row 7, Col j (D26)
static const int MOTOR_PIN = 27; // Row 6, Col j (D27)

static inline void setCues(bool active) {
  digitalWrite(LED_PIN, active ? HIGH : LOW);
  digitalWrite(MOTOR_PIN, active ? HIGH : LOW);
}

FogDetector fog;

 // ==================================================
 // FREEZING OF GAIT HAPTIC CUEING
 // ==================================================

 // Vibration timing parameters (milliseconds)
 const uint32_t LONG_BUZZ = 700;
 const uint32_t INITIAL_PAUSE = 300;
 const uint32_t SHORT_BUZZ = 150;
 const uint32_t STEP_INTERVAL = 900;
 const uint32_t REPEAT_FREEZE = 5000;
 const uint32_t RECOVERY_TIME = 5000;

 // Cueing state variables
 bool cueActive = false;
 bool recovering = false;

 uint32_t longBuzzStart = 0;
 uint32_t freezeEndTime = 0;

 // Called when freezing begins
 void startCueing() {
   cueActive = true;
   recovering = false;
   longBuzzStart = millis();

   Serial.println("Starting haptic cueing");
 }

 // Called when freezing ends
 void endFreezingCue() {
   recovering = true;
   freezeEndTime = millis();

   Serial.println("Freeze ended - continuing cadence");
 }

 // Called continuously from loop()
 void updateCueing() {
   if (!cueActive) {
     setCues(false);
     return;
   }

   uint32_t now = millis();

   // Stop cueing after the recovery period
   if (recovering &&
       now - freezeEndTime >= RECOVERY_TIME) {
     cueActive = false;
     recovering = false;
     setCues(false);
     return;
   }

   // If still freezing, restart with another long buzz
   if (fog.freezing() &&
       now - longBuzzStart >= REPEAT_FREEZE) {
     longBuzzStart = now;
     Serial.println("Still freezing - repeating long buzz");
   }

   uint32_t elapsed = now - longBuzzStart;

   // Phase 1: Long vibration
   if (elapsed < LONG_BUZZ) {
     setCues(true);
   }

   // Phase 2: Brief pause
   else if (elapsed < LONG_BUZZ + INITIAL_PAUSE) {
     setCues(false);
   }

   // Phase 3: Rhythmic short vibrations
   else {
     uint32_t rhythmTime =
       elapsed - LONG_BUZZ - INITIAL_PAUSE;

     uint32_t phase = rhythmTime % STEP_INTERVAL;

     setCues(phase < SHORT_BUZZ);
   }
 }

// ==================================================
// IMU BLE CONFIGURATION
// ==================================================

static const NimBLEUUID SVC_UUID(
  "0000ffe5-0000-1000-8000-00805f9a34fb"
);

static const NimBLEUUID NOTIFY_UUID(
  "0000ffe4-0000-1000-8000-00805f9a34fb"
);

static const NimBLEUUID WRITE_UUID(
  "0000ffe9-0000-1000-8000-00805f9a34fb"
);

// ==================================================
// SENSOR DATA
// ==================================================

struct Sample {
  uint32_t t_ms;
  float a[3];    // Acceleration in g
  float g[3];    // Gyroscope in degrees/second
  float ang[3];  // Angles in degrees
};

QueueHandle_t q = nullptr;

NimBLEClient* cli = nullptr;
NimBLERemoteCharacteristic* wchr = nullptr;

volatile bool connected = false;

// ==================================================
// BLE PACKET PARSING
// ==================================================

uint8_t pkt[20];
size_t n = 0;

void onNotify(
  NimBLERemoteCharacteristic*,
  uint8_t* d,
  size_t len,
  bool
) {
  for (size_t i = 0; i < len; i++) {
    uint8_t b = d[i];

    if (n == 0 && b != 0x55) {
      continue;
    }

    if (n == 1 && b != 0x61) {
      n = (b == 0x55) ? 1 : 0;
      if (n == 1) pkt[0] = 0x55;
      continue;
    }

    pkt[n++] = b;

    if (n == 20) {
      n = 0;

      int16_t v[9];

      for (int k = 0; k < 9; k++) {
        v[k] = (int16_t)(
          pkt[2 + 2 * k] |
          (pkt[3 + 2 * k] << 8)
        );
      }

      Sample s;
      s.t_ms = millis();

      for (int k = 0; k < 3; k++) {
        s.a[k] = v[k] / 32768.0f * 16.0f;
        s.g[k] = v[3 + k] / 32768.0f * 2000.0f;
        s.ang[k] = v[6 + k] / 32768.0f * 180.0f;
      }

      // Drop a sample if the queue is full.
      if (q != nullptr) {
        xQueueSend(q, &s, 0);
      }
    }
  }
}

// ==================================================
// CONNECT TO IMU
// ==================================================

bool connectSensor() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);

  Serial.println("Scanning for IMU...");

  NimBLEScanResults res = scan->getResults(5000, false);

  for (int i = 0; i < res.getCount(); i++) {
    const NimBLEAdvertisedDevice* dev = res.getDevice(i);

    if (dev->getName().rfind("WT", 0) != 0) {
      continue;
    }

    Serial.printf("Found IMU: %s\n", dev->getName().c_str());

    cli = NimBLEDevice::createClient();

    if (!cli->connect(dev)) {
      NimBLEDevice::deleteClient(cli);
      cli = nullptr;
      return false;
    }

    auto svc = cli->getService(SVC_UUID);

    if (!svc) {
      cli->disconnect();
      return false;
    }

    auto nchr = svc->getCharacteristic(NOTIFY_UUID);
    wchr = svc->getCharacteristic(WRITE_UUID);

    if (!nchr || !nchr->subscribe(true, onNotify)) {
      cli->disconnect();
      return false;
    }

    return true;
  }

  return false;
}

// ==================================================
// REQUEST 200 Hz SENSOR RATE
// ==================================================

void setRate200Hz() {
  if (!wchr) return;

  // These commands are from your original sketch.
  // Verify them against your sensor's manual.
  uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
  uint8_t rate[]   = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
  uint8_t save[]   = {0xFF, 0xAA, 0x00, 0x00, 0x00};

  wchr->writeValue(unlock, 5, false);
  delay(200);

  wchr->writeValue(rate, 5, false);
  delay(200);

  wchr->writeValue(save, 5, false);
  delay(200);
}

// ==================================================
// SETUP
// ==================================================

void setup() {
  // Initialize LED and Motor pins LOW immediately on boot
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  setCues(false);

  Serial.begin(921600);
  delay(1500);

  Serial.println();
  Serial.println("================================");
  Serial.println("       ESP32 FoG Detector");
  Serial.println("================================");

  Serial.printf("Model sampling rate: %d Hz\n", FOG_FS);
  Serial.printf("Window size: %d samples\n", FOG_WIN);
  Serial.printf("Hop size: %d samples\n", FOG_HOP);

  Serial.printf("Freeze weight: %.6f\n", FOG_W_FREEZE);
  Serial.printf("Loco weight: %.6f\n", FOG_W_LOCO);
  Serial.printf("Bias: %.6f\n", FOG_BIAS);

  q = xQueueCreate(64, sizeof(Sample));

  if (q == nullptr) {
    Serial.println("ERROR: Failed to create sample queue.");
    while (true) {
      delay(1000);
    }
  }

  NimBLEDevice::init("");

  // Incoming IMU rate requested by setRate200Hz().
  // FogDetector resamples this to the model's 64 Hz.
  fog.begin(200.0f, 0.0f, 1);

  Serial.println("Initialization complete.");
}

// ==================================================
// MAIN LOOP
// ==================================================

void loop() {
  if (!connected) {
    if (connectSensor()) {
      connected = true;

      setRate200Hz();

      // Sync output pins with detector state upon connection
      setCues(false);

      Serial.println("IMU connected.");
      Serial.println("FoG detection running.");
    } else {
      setCues(false);
      cueActive = false;
      recovering = false;
      Serial.println("IMU not found. Retrying...");
      delay(1000);
      return;
    }
  }

  if (cli && !cli->isConnected()) {
    connected = false;
    cueActive = false;
    setCues(false); // Fail-safe: turn off motor and LED if IMU disconnects
    recovering = false;
    Serial.println("IMU disconnected.");
    return;
  }

  static uint32_t sampleCount = 0;
  static uint32_t rateStart = 0;
  static uint32_t lastPrint = 0;

  if (rateStart == 0) {
    rateStart = millis();
    lastPrint = millis();
  }

  Sample s;

  // Wait up to 1 FreeRTOS tick for the first sample (wakes immediately on BLE notify),
  // then drain any remaining queued samples with 0 wait.
  if (xQueueReceive(q, &s, 1) == pdTRUE) {
    do {
      sampleCount++;

      FogEvent e = fog.update(
        s.a[0],
        s.a[1],
        s.a[2]
      );

      if (e == FOG_START) {
        startCueing(); // Turn LED and Motor HIGH immediately
        Serial.println(">>> FREEZE START <<<");
      } else if (e == FOG_END) {
        endFreezingCue(); // Turn LED and Motor LOW immediately
        Serial.println(">>> FREEZE END <<<");
      }
    } while (xQueueReceive(q, &s, 0) == pdTRUE);
  }
  // Update motor vibration pattern.
  updateCueing();
  uint32_t now = millis();

  // Report processed sample rate every five seconds.
  if (now - rateStart >= 5000) {
    float rateHz =
      sampleCount * 1000.0f / (now - rateStart);

    Serial.printf(
      "Processed input rate: %.1f Hz\n",
      rateHz
    );

    sampleCount = 0;
    rateStart = now;
  }

  // Print detector diagnostics twice per second.
  if (now - lastPrint >= 500) {
    lastPrint = now;

    Serial.printf(
      "Freeze feature: %.3f | "
      "Loco feature: %.3f | "
      "Model score: %.3f | "
      "Freezing: %s\n",
      fog.fFreeze,
      fog.fLoco,
      fog.score(),
      fog.freezing() ? "YES" : "NO"
    );
  }
}