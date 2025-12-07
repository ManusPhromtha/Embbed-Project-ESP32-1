#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define IR_RECV_PIN       5   // KY-022 OUT


uint8_t peerMac[] = { 0x80, 0xF3, 0xDA, 0x41, 0xC9, 0xD4 };
uint8_t theirState  = 0;  // last value received from the other board

uint8_t currentUnlockState = 0;

int detectCount = 0;
unsigned long window_gap = 0;
unsigned long detectStart = 0;
unsigned resultCounter = 0;
bool isFirstTime = true;
bool isDoorTrulyOpen = false;

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
    currentUnlockState = (theirState == 1) ? 1 : 0;
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
  Serial.println("IR detection system started.");
}



void loop() {
  if (currentUnlockState == 1 && isDoorTrulyOpen == false) {
    // Lock is open, Door is open?
    if (isFirstTime) window_gap = millis();
    isFirstTime = false;
    unsigned long now = millis();

    int unlockState = digitalRead(IR_RECV_PIN);

    if (digitalRead(IR_RECV_PIN) == LOW) {
      detectCount++;
    }

    // After 1000 ms, evaluate the result
    if (now-window_gap >= 2000) {
      // No detection in 2 seconds means door is truly open
      int result = (detectCount == 0) ? 1 : 0;

      Serial.print("IR result = ");
      Serial.print(result);
      Serial.print("  (detectCount = ");
      Serial.print(detectCount);
      Serial.println(")");
      
      if (result) {
        Serial.println("Door is truly open now.");
        isDoorTrulyOpen = true;
      }

      window_gap = now;
      detectCount = 0;
    }
    delay(1);
  } else if (isDoorTrulyOpen == true) {
    // Door is open, monitor IR receiver
    unsigned long now = millis();

    int unlockState = digitalRead(IR_RECV_PIN);

    if (digitalRead(IR_RECV_PIN) == LOW) {
      detectCount++;
    }

    // After 1000 ms, evaluate the result
    if (now-window_gap >= 1000) {

      int result = (detectCount >= 8) ? 1 : 0;

      Serial.print("IR result = ");
      Serial.print(result);
      Serial.print("  (detectCount = ");
      Serial.print(detectCount);
      Serial.println(")");
      
      if (result == 1) {
        if (resultCounter == 0) detectStart = now;
        resultCounter++;
        if (now - detectStart >= 5000 && resultCounter >= 5) {
          Serial.println("Door has been closed now.");
          currentUnlockState = 0;
          isDoorTrulyOpen = false;
          esp_err_t result = esp_now_send(peerMac, &currentUnlockState, sizeof(currentUnlockState));

          Serial.print("Sent currentUnlockState = ");
          Serial.print(currentUnlockState);
          Serial.print("  result = ");
          Serial.println(result == ESP_OK ? "OK" : "ERROR");
        } else if (now - detectStart >= 5000) {
          // Not continous for 5 seconds, reset
          Serial.println("IR detection unstable, resetting counters.");
          resultCounter = 0;
          detectStart = 0;
        }

        Serial.println(now - detectStart);
      } else {
        // Reset for next window
        resultCounter = 0;
        detectStart = 0;
      }
      window_gap = now;
      detectCount = 0;
    }
    delay(1);
  } else {
    // Door is closed, do nothing
    window_gap = 0;
    isFirstTime = true;
    delay(100);
  }
}

