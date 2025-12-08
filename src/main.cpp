#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define IR_RECV_PIN       5   // KY-022 OUT
#define BUZZER_PIN       21   // KY-022 OUT

#define LEAST_DETECT_COUNT 8 // Per 1000 ms
#define CLOSE_DETECT_TIME  4000
#define MAX_OPEN_TIME  30000

// ==================== CONFIG ====================
const char* ssid         = "TY7373_2.4G";
const char* password     = "0933897373";

const char* mqtt_server  = "mqtt.netpie.io";
const int   mqtt_port    = 1883;
const char* client_id    = "2e551e86-cfa3-41a2-b9b0-cc086ebf6b2c";
const char* device_token = "24R9C4Z1RkuqTU2FumuhCEA6crsZwFaG";
const char* device_secret= "AimrD3qZSbFx9qfGy7nMaNR7zQTrnq85";

// ==================== GLOBAL VARS ====================
WiFiClient   espClient;
PubSubClient client(espClient);
WiFiUDP      ntpUDP;
NTPClient    timeClient(ntpUDP, "pool.ntp.org", 7 * 3600); // GMT+7

// Params for MQTT message
int alarmCount = 0;

// ==================== WIFI ====================
void connectWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

// ==================== SHADOW UPDATE ====================
void updateShadow() {
  StaticJsonDocument<256> doc;

  JsonObject data = doc["data"].to<JsonObject>();
  JsonObject stat = data["stat"].to<JsonObject>();

  stat["alarmCount"] = alarmCount;

  char jsonBuffer[512];
  size_t n = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  if (n == 0) {
    Serial.println("Failed to serialize JSON (buffer too small?)");
    return;
  }

  client.publish("@shadow/data/update", jsonBuffer);
  Serial.print("Sent Update to NETPIE: ");
  Serial.println(jsonBuffer);
}


// ==================== MQTT CONNECT ====================
void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to NETPIE...");
    if (client.connect(client_id, device_token, device_secret)) {
      Serial.println(" connected");

      // Reset dashboard on connect
      updateShadow();
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}


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

  connectWiFi();

  timeClient.begin();
  timeClient.update();

  client.setServer(mqtt_server, mqtt_port);

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
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();
  timeClient.update();

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
          updateShadow();
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
        if (buzzerState) {
          alarmCount++;
        }
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
    alarmCount      = 0;
    isDoorTrulyOpen = false;
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

