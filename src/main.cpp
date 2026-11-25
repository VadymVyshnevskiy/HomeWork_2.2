#include <Arduino.h>

constexpr uint8_t RELAY_PIN = 4;
constexpr uint8_t CONTACT_PIN = 5;

constexpr uint8_t RELAY_ON = HIGH;
constexpr uint8_t RELAY_OFF = LOW;

constexpr int MEASUREMENTS = 10;
constexpr uint32_t SETTLE_US = 5000;
constexpr uint32_t TIMEOUT_US = 200000;
constexpr uint32_t PAUSE_MS = 500;

enum class Result { Ok, Timeout, Skipped };

struct Measurement {
  Result result;
  uint32_t delayUs;
  uint32_t bounceUs;
  uint32_t edges;
};

struct Stats {
  uint32_t delaySum = 0;
  uint32_t bounceSum = 0;
  int count = 0;

  void add(const Measurement &m) {
    if (m.result != Result::Ok) return;
    delaySum += m.delayUs;
    bounceSum += m.bounceUs;
    count++;
  }
};

portMUX_TYPE edgeMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t firstEdgeUs = 0;
volatile uint32_t lastEdgeUs = 0;
volatile uint32_t edgeCount = 0;

void IRAM_ATTR onContactEdge() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&edgeMux);
  if (edgeCount == 0) firstEdgeUs = now;
  lastEdgeUs = now;
  edgeCount++;
  portEXIT_CRITICAL_ISR(&edgeMux);
}

void resetEdges() {
  portENTER_CRITICAL(&edgeMux);
  edgeCount = 0;
  portEXIT_CRITICAL(&edgeMux);
}

Measurement switchRelay(uint8_t level, uint8_t expectedContact) {
  resetEdges();
  uint32_t start = micros();
  digitalWrite(RELAY_PIN, level);

  while (micros() - start < TIMEOUT_US) {
    portENTER_CRITICAL(&edgeMux);
    uint32_t edges = edgeCount;
    uint32_t first = firstEdgeUs;
    uint32_t last = lastEdgeUs;
    portEXIT_CRITICAL(&edgeMux);

    if (edges == 0 || micros() - last < SETTLE_US) continue;

    if (digitalRead(CONTACT_PIN) == expectedContact) {
      return {Result::Ok, first - start, last - first, edges};
    }
    resetEdges();
  }
  return {Result::Timeout, 0, 0, 0};
}

bool isContactOpen() {
  return digitalRead(CONTACT_PIN) == HIGH;
}

// After a timeout the contact state is unknown: put the relay back to OFF and check it really opened.
bool returnToIdle() {
  Serial.printf("  after timeout: contact %s, relay -> OFF\n", isContactOpen() ? "open" : "CLOSED");
  digitalWrite(RELAY_PIN, RELAY_OFF);
  delay(PAUSE_MS);
  const bool open = isContactOpen();
  Serial.printf("  idle check: contact %s\n", open ? "open, continuing" : "still CLOSED");
  return open;
}

void printMeasurement(const char *label, const Measurement &m) {
  if (m.result == Result::Timeout) {
    Serial.printf("  %s: TIMEOUT\n", label);
    return;
  }
  if (m.result == Result::Skipped) {
    Serial.printf("  %s: skipped, ON did not complete\n", label);
    return;
  }
  Serial.printf("  %s: %6lu us  (bounce %5lu us, %lu edges)\n", label,
                (unsigned long)m.delayUs, (unsigned long)m.bounceUs, (unsigned long)m.edges);
}

void printAverage(const char *label, const Stats &s) {
  if (s.count == 0) {
    Serial.printf("%s: no valid measurements\n", label);
    return;
  }
  Serial.printf("%s: %.1f us (%.2f ms), avg bounce %.1f us, valid %d/%d\n", label,
                (float)s.delaySum / s.count, (float)s.delaySum / s.count / 1000.0f,
                (float)s.bounceSum / s.count, s.count, MEASUREMENTS);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CONTACT_PIN, INPUT_PULLUP);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pinMode(RELAY_PIN, OUTPUT);
  delay(PAUSE_MS);

  Serial.println("\n=== Relay switching time ===");

  if (!isContactOpen()) {
    Serial.println("Contact is closed with the relay off. Probing:");
    for (uint8_t level : {HIGH, LOW, HIGH}) {
      digitalWrite(RELAY_PIN, level);
      delay(PAUSE_MS);
      Serial.printf("  IN=%s -> contact %s\n", level == HIGH ? "HIGH" : "LOW ",
                    isContactOpen() ? "open" : "CLOSED");
    }
    digitalWrite(RELAY_PIN, RELAY_OFF);
    return;
  }

  attachInterrupt(digitalPinToInterrupt(CONTACT_PIN), onContactEdge, CHANGE);

  Stats onStats, offStats;
  for (int i = 1; i <= MEASUREMENTS; i++) {
    Measurement on = switchRelay(RELAY_ON, LOW);
    Measurement off = {Result::Skipped, 0, 0, 0};
    if (on.result == Result::Ok) {
      delay(PAUSE_MS);
      off = switchRelay(RELAY_OFF, HIGH);
    }

    Serial.printf("#%d\n", i);
    printMeasurement("ON ", on);
    printMeasurement("OFF", off);

    onStats.add(on);
    offStats.add(off);

    if (on.result != Result::Ok || off.result != Result::Ok) {
      if (!returnToIdle()) {
        Serial.println("Contact did not open with the relay OFF, stopping the series.");
        break;
      }
    } else {
      delay(PAUSE_MS);
    }
  }

  detachInterrupt(digitalPinToInterrupt(CONTACT_PIN));

  Serial.println("\n--- Averages ---");
  printAverage("ON ", onStats);
  printAverage("OFF", offStats);
}

void loop() {}
