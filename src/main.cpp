#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define IR_RECV_PIN       5   // KY-022 OUT
#define BUZZER_PIN       21   // KY-022 OUT

#define LEAST_DETECT_COUNT 8 // Per 1000 ms
#define CLOSE_DETECT_TIME  4000
#define MAX_OPEN_TIME  10000


// ==================== MODULE SETUP ====================
uint8_t peerMac[] = { 0x80, 0xF3, 0xDA, 0x41, 0xC9, 0xD4 };
uint8_t theirState  = 0;  // last value received from the other board

uint8_t currentUnlockState = 0;

int detectCount = 0;
unsigned long window_gap = 0;
unsigned long detectStart = 0;
unsigned long openStart = 0;
unsigned resultCounter = 0;
bool isDoorTrulyOpen = false;

unsigned long previousMillis = 0;
const unsigned long interval = 500;  // 500 ms ON/OFF interval

bool buzzerState = false;

// Check send status
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
  if (status != ESP_NOW_SEND_SUCCESS) {
    //resent
    delay(1000);
    esp_err_t result = esp_now_send(peerMac, &currentUnlockState, sizeof(currentUnlockState));
  }
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

  // Wi-Fi setup
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

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}



void loop() {
  if (currentUnlockState == 1 && isDoorTrulyOpen == false) {
    // Lock is open, Door is open?
    if (window_gap == 0) window_gap = millis();
    unsigned long now = millis();

    int irState = digitalRead(IR_RECV_PIN);

    if (irState == LOW) {
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
    if (openStart == 0) openStart = now;

    int irState = digitalRead(IR_RECV_PIN);
    if (irState == LOW) {
      detectCount++;
    }

    // After 1000 ms, evaluate the result
    if (now-window_gap >= 1000) {

      int result = (detectCount >= LEAST_DETECT_COUNT) ? 1 : 0;

      Serial.print("IR result = ");
      Serial.print(result);
      Serial.print("  (detectCount = ");
      Serial.print(detectCount);
      Serial.println(")");
      
      if (result == 1) {
        // Turn off buzzer
        openStart = now;
        digitalWrite(BUZZER_PIN, LOW);
        
        if (resultCounter == 0) detectStart = now;
        resultCounter++;
        if (now - detectStart >= CLOSE_DETECT_TIME && resultCounter >= 5) {
          Serial.println("Door has been closed now.");
          currentUnlockState = 0;
          isDoorTrulyOpen = false;
          digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off
          esp_err_t result = esp_now_send(peerMac, &currentUnlockState, sizeof(currentUnlockState));

          Serial.print("Sent currentUnlockState = ");
          Serial.print(currentUnlockState);
          Serial.print("  result = ");
          Serial.println(result == ESP_OK ? "OK" : "ERROR");
          
        } else if (now - detectStart >= CLOSE_DETECT_TIME) {
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

    if (now - openStart >= MAX_OPEN_TIME && isDoorTrulyOpen) {
      unsigned long currentMillis = millis();

      // --- BUZZER TOGGLE TASK (runs in background) ---
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Toggle buzzer state
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
      }
    }  
    delay(1);
  } else {
    // Door is closed, do nothing
    window_gap     = 0;
    openStart      = 0;
    detectStart    = 0;
    detectCount    = 0;
    resultCounter  = 0;
    isDoorTrulyOpen = false;
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

