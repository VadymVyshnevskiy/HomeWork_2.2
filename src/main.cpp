#include <Arduino.h>

constexpr uint8_t RELAY_PIN = 4;
constexpr uint8_t CONTACT_PIN = 5;

constexpr uint8_t RELAY_ON = HIGH;
constexpr uint8_t RELAY_OFF = LOW;

constexpr int MEASUREMENTS = 10;
constexpr uint32_t SETTLE_US = 5000;
constexpr uint32_t TIMEOUT_US = 200000;
constexpr uint32_t PAUSE_MS = 500;

struct Measurement {
  bool ok;
  uint32_t delayUs;
  uint32_t bounceUs;
  uint32_t edges;
};

struct Stats {
  uint32_t delaySum = 0;
  uint32_t bounceSum = 0;
  int count = 0;

  void add(const Measurement &m) {
    if (!m.ok) return;
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
      return {true, first - start, last - first, edges};
    }
    resetEdges();
  }
  return {false, 0, 0, 0};
}

void printMeasurement(const char *label, const Measurement &m) {
  if (!m.ok) {
    Serial.printf("  %s: TIMEOUT\n", label);
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

  if (digitalRead(CONTACT_PIN) != HIGH) {
    Serial.println("Contact is closed with the relay off. Probing:");
    for (uint8_t level : {HIGH, LOW, HIGH}) {
      digitalWrite(RELAY_PIN, level);
      delay(PAUSE_MS);
      Serial.printf("  IN=%s -> contact %s\n", level == HIGH ? "HIGH" : "LOW ",
                    digitalRead(CONTACT_PIN) == LOW ? "CLOSED" : "open");
    }
    digitalWrite(RELAY_PIN, RELAY_OFF);
    return;
  }

  attachInterrupt(digitalPinToInterrupt(CONTACT_PIN), onContactEdge, CHANGE);

  Stats onStats, offStats;
  for (int i = 1; i <= MEASUREMENTS; i++) {
    Measurement on = switchRelay(RELAY_ON, LOW);
    delay(PAUSE_MS);
    Measurement off = switchRelay(RELAY_OFF, HIGH);
    delay(PAUSE_MS);

    onStats.add(on);
    offStats.add(off);

    Serial.printf("#%d\n", i);
    printMeasurement("ON ", on);
    printMeasurement("OFF", off);
  }

  detachInterrupt(digitalPinToInterrupt(CONTACT_PIN));

  Serial.println("\n--- Averages ---");
  printAverage("ON ", onStats);
  printAverage("OFF", offStats);
}

void loop() {}
