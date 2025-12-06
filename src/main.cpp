#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define IR_LED_PIN        23   // KY-005 signal
#define IR_RECV_PIN       5   // KY-022 OUT
#define LED_PIN       21   // LED PIN

const int IR_CHANNEL = 0;
const int IR_FREQ    = 16;
const int IR_RES     = 8;

unsigned long windowStart = 0;
int detectCount = 0;

uint8_t peerMac[] = { 0x80, 0xF3, 0xDA, 0x41, 0xC9, 0xD4 };
uint8_t myState     = 0;  // what THIS board is sending (0/1)
uint8_t theirState  = 0;  // last value received from the other board

bool doorState = false;

// Optional: check send status
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// Called when *this* ESP32 receives data from the peer
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len >= 1) {
    theirState = incomingData[0];
    Serial.print("Got theirState = ");
    Serial.println(theirState);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1) Wi-Fi must be in STA mode for ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.print("This board MAC: ");
  Serial.println(WiFi.macAddress());

  // 2) Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // 3) Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // 4) Add peer (the other ESP32)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;      // use current WiFi channel
  peerInfo.encrypt = false;  // no encryption for now

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW two-way node ready.");

  ledcSetup(IR_CHANNEL, IR_FREQ, IR_RES);
  ledcAttachPin(IR_LED_PIN, IR_CHANNEL);
  ledcWrite(IR_CHANNEL, 128);

  pinMode(IR_RECV_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  windowStart = millis();
  Serial.println("IR detection system started.");
}

void loop() {
  static unsigned long lastToggle = 0;
  unsigned long now = millis();

  if (now - lastToggle >= 2000) {
    lastToggle = now;

    // Toggle between 0 and 1
    myState = (myState == 0) ? 1 : 0;

    esp_err_t result = esp_now_send(peerMac, &myState, sizeof(myState));

    Serial.print("Sent myState = ");
    Serial.print(myState);
    Serial.print("  result = ");
    Serial.println(result == ESP_OK ? "OK" : "ERROR");
  }



  


  // unsigned long now = millis();

  // int unlockState = digitalRead(IR_RECV_PIN);

  // if (digitalRead(IR_RECV_PIN) == LOW) {
  //   detectCount++;
  // }

  // // After 1000 ms, evaluate the result
  // if (now - windowStart >= 1000) {

  //   int result = (detectCount >= 8) ? 1 : 0;

  //   Serial.print("IR result = ");
  //   Serial.print(result);
  //   Serial.print("  (detectCount = ");
  //   Serial.print(detectCount);
  //   Serial.println(")");
  //   digitalWrite(LED_PIN, HIGH);
  //   detectCount = 0;
  //   windowStart = now;
  // }
  // delay(1);
}

