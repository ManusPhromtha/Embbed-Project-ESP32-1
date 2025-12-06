#include <Arduino.h>

#define IR_LED_PIN        23   // KY-005 signal
#define IR_RECV_PIN       5   // KY-022 OUT
#define LED_PIN       21   // LED PIN

const int IR_CHANNEL = 0;
const int IR_FREQ    = 16;
const int IR_RES     = 8;

unsigned long windowStart = 0;
int detectCount = 0;

void setup() {
  Serial.begin(115200);

  ledcSetup(IR_CHANNEL, IR_FREQ, IR_RES);
  ledcAttachPin(IR_LED_PIN, IR_CHANNEL);
  ledcWrite(IR_CHANNEL, 128);

  pinMode(IR_RECV_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  windowStart = millis();
  Serial.println("IR detection system started.");
}

void loop() {
  unsigned long now = millis();

  int unlockState = digitalRead(IR_RECV_PIN);

  if (digitalRead(IR_RECV_PIN) == LOW) {
    detectCount++;
  }

  // After 1000 ms, evaluate the result
  if (now - windowStart >= 1000) {

    int result = (detectCount >= 8) ? 1 : 0;

    Serial.print("IR result = ");
    Serial.print(result);
    Serial.print("  (detectCount = ");
    Serial.print(detectCount);
    Serial.println(")");
    digitalWrite(LED_PIN, HIGH);
    detectCount = 0;
    windowStart = now;
  }
  delay(1);
}