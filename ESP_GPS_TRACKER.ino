/*
This is a code for GPS Tracker with Accident Detection alert system using MPU6050 gyro.
Owner name: Susovan Das
oweer's mail : susovandas805@gmail.com
Date Created : 14-02-2025
version : v0.0.1
*/


// Required Libraries.
#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <esp_sleep.h>

// Some constants and Impt. Variables.
#define LED_PIN 2                      // Status LED  
#define BUZZER_PIN 25                  // Buzzer connected to GPIO 25
#define ABORT_PIN 32                   // S.O.S Abort Button GPIO  
#define COOLDOWN_TIME 30000            // 30 seconds cooldown  
#define CRASH_THRESHOLD 5.0            // G-force threshold for crash detection
#define RETRY_LIMIT 3                  // Retry sending SMS up to 3 times
#define EEPROM_SIZE 240                // 10 GPS entries (each lat, lng, time is 24 bytes)
#define SLEEP_DURATION 30              // Deep sleep for 30 seconds
#define WAKEUP_PIN GPIO_NUM_33         // Optional wake-up button

// MPU6050 Registers
#define MPU6050_ADDR 0x68
#define ACCEL_XOUT_H 0x3B

// Emergency Contacts
const char* EMERGENCY_CONTACTS[] = {"+91 XXXXX XXXX", "+91 YYYYY YYYYY"};
const int NUM_CONTACTS = sizeof(EMERGENCY_CONTACTS) / sizeof(EMERGENCY_CONTACTS[0]);

// Firebase Settings
const char* firebaseHost = "your-project-id.firebaseio.com";        // Replace with your Firebase project
const char* firebasePath = "/tracker/data.json";                    // Firebase path

// Last known GPS location
double lastLat = 0.0, lastLng = 0.0;

RTC_DATA_ATTR int wakeUpCounter = 0;                                // Counter survives deep sleep

// GPS and SIM800L Serial Ports
HardwareSerial gpsSerial(1);                                        // GPS module on UART1
HardwareSerial sim800Serial(2);                                     // SIM800L module on UART2

TinyGPSPlus gps;

void setup() {
    Serial.begin(115200); 
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);     // Neo6m GPS TX->16, RX->17
    sim800Serial.begin(9600, SERIAL_8N1, 27, 26);  //   SIM800L TX->27, RX->26
    
    Wire.begin(); 
    initMPU6050();
    
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(ABORT_PIN, INPUT_PULLUP);  // Abort button (active LOW)
    
    digitalWrite(LED_PIN, LOW);        // Accident LED initially OFF
    digitalWrite(BUZZER_PIN, LOW);     // Buzzer OFF initially
    
    EEPROM.begin(EEPROM_SIZE);         // Initialize EEPROM

    esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1e6);  // Sleep for 30 sec
    esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, LOW);        // Wake-up button
    
    delay(1000);
    Serial.println("Setup complete.");
}

void loop() {
    float accX, accY, accZ;
    readMPU6050(accX, accY, accZ);
    
    double latitude = 0.0, longitude = 0.0;
    uint32_t timestamp = getUnixTime();  // Get current Unix time
    
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            lastLat = gps.location.lat();
            lastLng = gps.location.lng();
            latitude = lastLat;
            longitude = lastLng;
            saveGPSLocation(lastLat, lastLng, timestamp);  // Save in EEPROM
            break;
        }
    }

    // Check for crash
    float totalGForce = sqrt(accX * accX + accY * accY + accZ * accZ);
    if (totalGForce >= CRASH_THRESHOLD) {
        Serial.println("CRASH DETECTED! Sending S.O.S...");
        activateBuzzer();
        digitalWrite(LED_PIN, HIGH);

        // cooldown delay before sending sos.
        unsigned long startTime = millis();
        bool aborted = false;

        while (millis() - startTime < COOLDOWN_TIME) {
            if (digitalRead(ABORT_PIN) == LOW) {  // If button is pressed
                Serial.println("S.O.S ABORTED!");
                digitalWrite(BUZZER_PIN, LOW);
                digitalWrite(LED_PIN, LOW);
                aborted = true;
                break;
            }
            beepBuzzer();  // Keep beeping
            delay(100);
        }

        digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer after cooldown
        digitalWrite(LED_PIN, LOW); // Turn off the indicator.
        
        if (!aborted) {
            Serial.println("S.O.S Sending...");
            sendSOSMessage();
            return; // Dont't go to sleep after crash
        }
    }
    
    
    // Prepare JSON payload
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["lat"] = latitude;
    jsonDoc["lng"] = longitude;
    jsonDoc["accX"] = accX;
    jsonDoc["accY"] = accY;
    jsonDoc["accZ"] = accZ;
    
    char jsonBuffer[200];
    serializeJson(jsonDoc, jsonBuffer);
    Serial.println("Sending data to Firebase...");
    sendDataToFirebase(jsonBuffer);

    delay(2000); // Delay before sending next data
    
    wakeUpCounter++;
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
}

void initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // Power management register
    Wire.write(0);  // Wake up MPU6050
    Wire.endTransmission(true);
}

void readMPU6050(float &ax, float &ay, float &az) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    ax = (Wire.read() << 8 | Wire.read()) / 16384.0; // Scale factor for ±2g
    ay = (Wire.read() << 8 | Wire.read()) / 16384.0;
    az = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

void sendDataToFirebase(const char* payload) {
    sim800Serial.println("AT"); // Check if SIM800L is responding
    delay(100);
    sim800Serial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
    delay(100);
    sim800Serial.println("AT+SAPBR=3,1,\"APN\",\"your_apn\""); // Set your APN
    delay(100);
    sim800Serial.println("AT+SAPBR=1,1"); // Open GPRS connection
    delay(3000);
    
    // Start HTTP session
    sim800Serial.println("AT+HTTPINIT");
    delay(1000);
    sim800Serial.println("AT+HTTPPARA=\"CID\",1");
    delay(1000);
    
    String url = "AT+HTTPPARA=\"URL\",\"http://" + String(firebaseHost) + firebasePath + "\"";
    sim800Serial.println(url);
    delay(1000);
    
    sim800Serial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    delay(1000);
    
    sim800Serial.println("AT+HTTPDATA=" + String(strlen(payload)) + ",10000");
    delay(1000);
    sim800Serial.println(payload);
    delay(3000);
    
    sim800Serial.println("AT+HTTPACTION=1"); // Send POST request
    delay(5000);
    
    sim800Serial.println("AT+HTTPREAD"); // Read response
    delay(3000);
    
    sim800Serial.println("AT+HTTPTERM"); // Terminate HTTP session
    delay(1000);
    
    Serial.println("Data sent to Firebase.");
}

void saveGPSLocation(double lat, double lng, uint32_t time) {
    static int index = 0;
    index = (index + 1) % 10; // rotating in 10 space

    EEPROM.put(index * 24, lat);
    EEPROM.put(index * 24 + 8, lng);
    EEPROM.put(index * 24 + 16, time);
    EEPROM.commit(); // Save data permanently
}

void retrieveLastGPSLocation(double &lat, double &lng, uint32_t &time) {
    for (int i = 9; i >= 0; i--) {
        EEPROM.get(i * 24, lat);
        EEPROM.get(i * 24 + 8, lng);
        EEPROM.get(i * 24 + 16, time);
        if (lat != 0 && lng != 0) break;
    }
}

uint32_t getUnixTime() {
    return millis() / 1000 + 1700000000;  // Fake Unix timestamp (adjust for RTC)
}

void sendSOSMessage() {
    double lat, lng;
    uint32_t time;
    retrieveLastGPSLocation(lat, lng, time);

    for (int i = 0; i < NUM_CONTACTS; i++) {
        String message = "S.O.S! Crash at: " + String(lat, 6) + ", " + String(lng, 6) + " at time " + String(time);
        bool success = sendSMS(EMERGENCY_CONTACTS[i], message.c_str());

        int retries = 0;
        while (!success && retries < RETRY_LIMIT) {
            Serial.println("Retrying SMS...");
            success = sendSMS(EMERGENCY_CONTACTS[i], message.c_str());
            retries++;
        }

        if (!success) Serial.println("Failed to send SMS after retries.");
    }
}

bool sendSMS(const char* phoneNumber, const char* message) {
    Serial.print("Sending SMS to ");
    Serial.println(phoneNumber);
    
    sim800Serial.println("AT+CMGF=1"); // Set SMS mode to text
    delay(1000);
    
    sim800Serial.print("AT+CMGS=\"");
    sim800Serial.print(phoneNumber);
    sim800Serial.println("\"");
    delay(1000);
    
    sim800Serial.print(message);
    delay(1000);
    
    sim800Serial.write(26); // End SMS with CTRL+Z
    delay(5000);
    
    String response = "";
    while (sim800Serial.available()) {
        response += (char)sim800Serial.read();
    }
    
    if (response.indexOf("OK") != -1) {
        Serial.println("SMS sent successfully!");
        return true;
    } else {
        Serial.println("SMS failed!");
        return false;
    }
}

void activateBuzzer() {
    for (int i = 0; i < 5; i++) {  // Initial alert
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        delay(200);
    }
}

void beepBuzzer() {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
}
