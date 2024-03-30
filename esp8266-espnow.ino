#include <espnow.h>
#include <ESP8266WiFi.h>
#include <stdint.h>

#define CHANNEL 1
#define LED 12

// Structure to match the sender's data structure
typedef struct struct_message {
    int32_t x; // Using 4 bytes explicitly
    int32_t y; // Using 4 bytes explicitly
    uint8_t xReverse; // Using 1 byte explicitly
    uint8_t yReverse; // Using 1 byte explicitly
    uint8_t padding[2];
} __attribute__((packed)) struct_message;

// Init ESP Now with fallback
void InitESPNow() {
    WiFi.disconnect();
    if (esp_now_init() == ERR_OK) {
        Serial.println("ESPNow Init Success");
    } else {
        Serial.println("ESPNow Init Failed");
        ESP.restart();
    }
}

// config AP SSID
void configDeviceAP() {
    String Prefix = "Slave:";
    String Mac = WiFi.macAddress();
    String SSID = Prefix + Mac;
    String Password = "123456789";
    bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
    if (!result) {
        Serial.println("AP Config failed.");
    } else {
        Serial.println("AP Config Success. Broadcasting with AP: " + SSID);
    }
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
    Serial.print("Expected size: ");
    Serial.println(sizeof(struct_message)); // Expected size based on your struct definition
    Serial.print("Received size: ");
    Serial.println(len); // Actual received data size

    if (len == sizeof(struct_message)) {
        struct_message* msg = reinterpret_cast<struct_message*>(incomingData);

        Serial.print("X: ");
        Serial.print(msg->x);
        if(msg->x>180){
          digitalWrite(LED, HIGH);
          
        }else {
          digitalWrite(LED, LOW); 
        }
        Serial.print(", Y: ");
        Serial.print(msg->y);
        Serial.print(", X Reverse: ");
        Serial.print(msg->xReverse ? "True" : "False");
        Serial.print(", Y Reverse: ");
        Serial.println(msg->yReverse ? "True" : "False");
        

    } else {
        Serial.println("Received data size does not match");
    }
}



void setup() {
    
    Serial.begin(115200);
    pinMode(LED, OUTPUT); 
    //Serial.println("ESPNow/Basic/Slave Example");
    WiFi.mode(WIFI_AP);
    configDeviceAP();
    //Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
    InitESPNow();
    esp_now_register_recv_cb(OnDataRecv);
}
// void setup(){
//   pinMode(12, OUTPUT); 
//   Serial.begin(115200);
// }

void loop() {
    // Keep the loop empty as there's nothing to do here
    // digitalWrite(12, HIGH);
    // Serial.println("High");
    // delay(1000);
    // digitalWrite(12, LOW);
    // Serial.println("Low");
    // delay(2000);
}
