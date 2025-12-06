#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define IR_RECV_PIN       5   // KY-022 OUT

unsigned long windowStart = 0;
int detectCount = 0;

uint8_t peerMac[] = { 0x80, 0xF3, 0xDA, 0x41, 0xC9, 0xD4 };
uint8_t theirState  = 0;  // last value received from the other board

uint8_t currentDoorState = 0;

// Check send status
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
    currentDoorState = (theirState == 1) ? 1 : 0;
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

  pinMode(IR_RECV_PIN, INPUT);
  windowStart = millis();
  Serial.println("IR detection system started.");
}

void loop() {
  if (currentDoorState == 1) {
    // Door is open, monitor IR receiver
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
      
      if (result) {
        currentDoorState = 0;

        esp_err_t result = esp_now_send(peerMac, &currentDoorState, sizeof(currentDoorState));

        Serial.print("Sent currentDoorState = ");
        Serial.print(currentDoorState);
        Serial.print("  result = ");
        Serial.println(result == ESP_OK ? "OK" : "ERROR");
      }
      detectCount = 0;
      windowStart = now;
    }
    delay(1);
  } else {
    // Door is closed, do nothing
    delay(100);
  }
}

