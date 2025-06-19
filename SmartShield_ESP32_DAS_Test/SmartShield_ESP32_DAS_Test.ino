/*
  Smart Shield DAS Test Version - Real Sensor Readings
  Tests storage capabilities and sensor functionality
  Features: Real sensor readings with fallbacks, Storage to SPIFFS, USB/BLE data access
  Python script handles detailed logging output
*/

#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <WebServer.h>

// BLE Configuration
#define SERVICE_UUID        "12345678-1234-5678-9abc-def123456789"
#define CHARACTERISTIC_UUID "87654321-4321-8765-cba9-9876543210fe"
#define DEVICE_NAME         "SensorShield_ESP32"

// Sensor Configuration
#define LC709203F_ADDRESS 0x0B    // Fuel gauge
#define SHT30_ADDRESS 0x44        // Common SHT30 I²C address
#define SHT31_ADDRESS 0x45        // Alternative SHT31 address
#define BATTERY_ADC_PIN 35        // Fallback ADC pin for battery voltage
#define TAMPER_PIN 14             // Digital pin for tamper switch
#define TEMP_FALLBACK_PIN 36      // ADC pin for temperature fallback (thermistor)

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Sensor availability flags
bool fuelGaugeAvailable = false;
bool shtSensorAvailable = false;
uint8_t shtSensorAddress = 0;
bool tamperPinAvailable = true;  // Assume digital pin is always available

// Sensor data structure
struct SensorData {
  unsigned long timestamp;
  float temperature;
  float humidity;
  float batteryVoltage;
  int tamperState;
  int bootNumber;
};

// Storage configuration
const char* dataFileName = "/sensor_data.json";
const char* bootCountFileName = "/boot_count.txt";
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL = 5000; // Log every 5 seconds
int bootCount = 1;

// WiFi access point configuration
const char* apSSID = "SensorShield";
const char* apPassword = "sensorpass";
WebServer server(80);


// Function prototypes
bool initSPIFFS();
bool initBLE();
bool initSensors();
float readBatteryVoltage();
float readTemperature();
float readHumidity();
int readTamperState();
void logSensorData(const SensorData& data);
void broadcastBLEData(const SensorData& data);
String readStoredData(int maxEntries);
void handleSerialCommands();
void handleRoot();
void handleData();
void handleStatus();

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("STATUS:BLE_CONNECTED");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("STATUS:BLE_DISCONNECTED");
    }
};

// BLE Characteristic Callbacks for command handling
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      String value = pCharacteristic->getValue().c_str();
      
      if (value == "DUMP_ALL") {
        // Send stored data dump via BLE
        pCharacteristic->setValue("DUMP_START");
        pCharacteristic->notify();
        delay(50);
        
        File file = SPIFFS.open(dataFileName, FILE_READ);
        if (file) {
          while (file.available()) {
            String line = file.readStringUntil('\n');
            line.trim();
            if (line.length() > 0) {
              pCharacteristic->setValue(line.c_str());
              pCharacteristic->notify();
              delay(50);  // Small delay to prevent BLE overflow
            }
          }
          file.close();
        }
        
        pCharacteristic->setValue("DUMP_END");
        pCharacteristic->notify();
      }
    }
};

bool initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR:SPIFFS_MOUNT_FAILED");
    return false;
  }

  Serial.println("STATUS:SPIFFS_INITIALIZED");
  Serial.println("SPIFFS initialized");

  // Read and increment boot count
  File bootFile = SPIFFS.open(bootCountFileName, FILE_READ);
  if (bootFile) {
    bootCount = bootFile.parseInt();
    bootFile.close();
  }
  
  bootCount++;
  
  bootFile = SPIFFS.open(bootCountFileName, FILE_WRITE);
  if (bootFile) {
    bootFile.println(bootCount);
    bootFile.close();
  }
  
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes  = SPIFFS.usedBytes();

  Serial.printf("Total space: %u bytes\n", totalBytes);
  Serial.printf("Storage Used: %u/%u bytes\n", usedBytes, totalBytes);
  Serial.printf("SPIFFS:TOTAL:%u,USED:%u,FREE:%u,BOOT:%d\n",
                totalBytes, usedBytes,
                totalBytes - usedBytes, bootCount);
  return true;
}

bool initBLE() {
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.printf("BLE:STARTED:%s\n", DEVICE_NAME);
  return true;
}

bool initSensors() {
  Wire.begin();
  delay(100);
  
  // Check for LC709203F fuel gauge
  Wire.beginTransmission(LC709203F_ADDRESS);
  if (Wire.endTransmission() == 0) {
    fuelGaugeAvailable = true;
    Serial.println("SENSOR:FUEL_GAUGE:AVAILABLE");
  } else {
    Serial.println("SENSOR:FUEL_GAUGE:NOT_FOUND");
  }
  
  // Check for SHT30/SHT31 temperature/humidity sensor
  Wire.beginTransmission(SHT30_ADDRESS);
  if (Wire.endTransmission() == 0) {
    shtSensorAvailable = true;
    shtSensorAddress = SHT30_ADDRESS;
    Serial.println("SENSOR:SHT30:AVAILABLE");
  } else {
    Wire.beginTransmission(SHT31_ADDRESS);
    if (Wire.endTransmission() == 0) {
      shtSensorAvailable = true;
      shtSensorAddress = SHT31_ADDRESS;
      Serial.println("SENSOR:SHT31:AVAILABLE");
    } else {
      Serial.println("SENSOR:SHT:NOT_FOUND");
    }
  }
  
  // Initialize tamper pin
  pinMode(TAMPER_PIN, INPUT_PULLUP);
  Serial.printf("SENSOR:TAMPER:PIN_%d:INITIALIZED\n", TAMPER_PIN);
  
  return true;
}

float readBatteryVoltage() {
  if (fuelGaugeAvailable) {
    // Try to read from LC709203F fuel gauge
    Wire.beginTransmission(LC709203F_ADDRESS);
    Wire.write(0x04); // Cell voltage register
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(LC709203F_ADDRESS, 2);
      if (Wire.available() >= 2) {
        uint16_t voltage = Wire.read() | (Wire.read() << 8);
        return voltage / 1000.0; // Convert mV to V
      }
    }
  }
  
  // Fallback to ADC reading (with voltage divider calculation)
  int adcValue = analogRead(BATTERY_ADC_PIN);
  // Assuming 3.3V reference and 2:1 voltage divider
  float voltage = (adcValue / 4095.0) * 3.3 * 2.0;
  
  // If no real battery connected, return 0 instead of simulated value
  if (voltage < 2.5) {  // Below minimum Li-Po voltage
    return 0.0;
  }
  
  return voltage;
}

float readTemperature() {
  if (shtSensorAvailable) {
    // Read from SHT30/SHT31 sensor
    Wire.beginTransmission(shtSensorAddress);
    Wire.write(0x2C);  // High repeatability measurement
    Wire.write(0x06);
    if (Wire.endTransmission() == 0) {
      delay(15);  // Wait for measurement
      
      Wire.requestFrom(shtSensorAddress, 6);
      if (Wire.available() >= 6) {
        uint8_t data[6];
        for (int i = 0; i < 6; i++) {
          data[i] = Wire.read();
        }
        
        // Convert temperature (first 3 bytes: temp MSB, temp LSB, temp CRC)
        uint16_t tempRaw = (data[0] << 8) | data[1];
        float temperature = -45.0 + (175.0 * tempRaw / 65535.0);
        
        // Sanity check
        if (temperature > -40 && temperature < 85) {
          return temperature;
        }
      }
    }
  }
  
  // Fallback to thermistor reading on ADC pin
  int adcValue = analogRead(TEMP_FALLBACK_PIN);
  if (adcValue > 0) {
    // Simple thermistor calculation (10K NTC, 3.3V, 10K pullup)
    float voltage = (adcValue / 4095.0) * 3.3;
    float resistance = (10000.0 * voltage) / (3.3 - voltage);
    // Simplified B-parameter equation for 10K NTC (B=3950)
    float temperature = 1.0 / ((1.0 / 298.15) + (1.0 / 3950.0) * log(resistance / 10000.0)) - 273.15;
    
    if (temperature > -10 && temperature < 60) {  // Reasonable range
      return temperature;
    }
  }
  
  // Final fallback: Return NaN to indicate no sensor
  return NAN;
}

float readHumidity() {
  if (shtSensorAvailable) {
    // Read from SHT30/SHT31 sensor (humidity is in the same reading as temperature)
    Wire.beginTransmission(shtSensorAddress);
    Wire.write(0x2C);  // High repeatability measurement
    Wire.write(0x06);
    if (Wire.endTransmission() == 0) {
      delay(15);  // Wait for measurement
      
      Wire.requestFrom(shtSensorAddress, 6);
      if (Wire.available() >= 6) {
        uint8_t data[6];
        for (int i = 0; i < 6; i++) {
          data[i] = Wire.read();
        }
        
        // Convert humidity (last 3 bytes: hum MSB, hum LSB, hum CRC)
        uint16_t humRaw = (data[3] << 8) | data[4];
        float humidity = 100.0 * humRaw / 65535.0;
        
        // Sanity check
        if (humidity >= 0 && humidity <= 100) {
          return humidity;
        }
      }
    }
  }
  
  // No reliable humidity fallback available
  return NAN;
}

int readTamperState() {
  if (tamperPinAvailable) {
    // Read digital tamper pin (active low with pullup)
    int pinState = digitalRead(TAMPER_PIN);
    
    // Simple tamper detection:
    // 0 = tamper detected (pin pulled low)
    // 1 = normal state (pin high due to pullup)
    return pinState == LOW ? 1 : 0;
  }
  
  // No fallback sensor available - return error state
  return -1;  // Indicate tamper sensor not available
}

void broadcastBLEData(const SensorData& data) {
  if (pCharacteristic != NULL) {
    DynamicJsonDocument doc(250);
    doc["timestamp"] = data.timestamp;
    
    // Only include valid sensor readings
    if (!isnan(data.temperature)) {
      doc["temperature"] = data.temperature;
    }
    if (!isnan(data.humidity)) {
      doc["humidity"] = data.humidity;
    }
    
    doc["battery_voltage"] = data.batteryVoltage;
    doc["tamper_state"] = data.tamperState;
    doc["boot_number"] = data.bootNumber;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    pCharacteristic->setValue(jsonString.c_str());
    if (deviceConnected) {
      pCharacteristic->notify();
    }
  }
}

void logSensorData(const SensorData& data) {
  File file = SPIFFS.open(dataFileName, FILE_APPEND);
  if (!file) {
    Serial.println("ERROR:FILE_WRITE_FAILED");
    return;
  }
  
  // Create JSON object
  DynamicJsonDocument doc(250);
  doc["timestamp"] = data.timestamp;
  
  // Only log valid sensor readings
  if (!isnan(data.temperature)) {
    doc["temperature"] = data.temperature;
  } else {
    doc["temperature"] = nullptr;  // Explicitly null for missing sensor
  }
  
  if (!isnan(data.humidity)) {
    doc["humidity"] = data.humidity;
  } else {
    doc["humidity"] = nullptr;  // Explicitly null for missing sensor
  }
  
  doc["battery_voltage"] = data.batteryVoltage;
  doc["tamper_state"] = data.tamperState;
  doc["boot_number"] = data.bootNumber;
  
  String jsonString;
  serializeJson(doc, jsonString);
  file.println(jsonString);
  file.close();

  // Simplified output for Python parsing
  Serial.println("Data logged: " + jsonString);
  Serial.printf("Storage Used: %u/%u bytes\n", SPIFFS.usedBytes(), SPIFFS.totalBytes());
}

String readStoredData(int maxEntries = 50) {
  File file = SPIFFS.open(dataFileName, FILE_READ);
  if (!file) {
    return "[]";
  }
  
  String jsonArray = "[";
  String line;
  int entryCount = 0;
  
  while (file.available() && entryCount < maxEntries) {
    line = file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      if (entryCount > 0) jsonArray += ",";
      jsonArray += line;
      entryCount++;
    }
  }
  
  jsonArray += "]";
  file.close();
  
  return jsonArray;
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "DUMP_ALL") {
      Serial.println("DUMP_START");
      
      File file = SPIFFS.open(dataFileName, FILE_READ);
      if (file) {
        while (file.available()) {
          String line = file.readStringUntil('\n');
          line.trim();
          if (line.length() > 0) {
            Serial.println(line);  // Send each JSON line
          }
        }
        file.close();
      }
      
      Serial.println("DUMP_END");
    }
    else if (command == "STATUS") {
      Serial.printf("STATUS:UPTIME:%lu,FREE_MEM:%d,STORAGE:%u/%u,BLE:%s\n",
                   millis()/1000, ESP.getFreeHeap(), 
                   SPIFFS.usedBytes(), SPIFFS.totalBytes(),
                   deviceConnected ? "CONNECTED" : "ADVERTISING");
    }
    else if (command == "SENSOR_STATUS") {
      Serial.printf("SENSORS:FUEL_GAUGE:%s,SHT:%s,TAMPER:%s\n",
                   fuelGaugeAvailable ? "OK" : "MISSING",
                   shtSensorAvailable ? "OK" : "MISSING",
                   tamperPinAvailable ? "OK" : "MISSING");
    }
    else if (command == "CLEAR_DATA") {
      if (SPIFFS.remove(dataFileName)) {
        Serial.println("STATUS:DATA_CLEARED");
      } else {
        Serial.println("ERROR:CLEAR_FAILED");
      }
    }
  }
}

void handleRoot() {
  server.send(200, "text/plain", "Sensor Shield");
}

void handleData() {
  String json = readStoredData(100);
  server.send(200, "application/json", json);
}

void handleStatus() {
  DynamicJsonDocument doc(200);
  doc["boot_count"] = bootCount;
  doc["uptime_seconds"] = millis() / 1000;
  doc["free_memory"] = ESP.getFreeHeap();
  doc["spiffs_used"] = SPIFFS.usedBytes();
  doc["spiffs_total"] = SPIFFS.totalBytes();
  doc["ble_connected"] = deviceConnected;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("SYSTEM:STARTUP");
  
  // Initialize components
  initSPIFFS();
  initBLE();
  initSensors();

  // Start WiFi access point
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.softAP(apSSID, apPassword);

  // HTTP server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/status", handleStatus);
  server.begin();

  Serial.println("SYSTEM:READY");
}

void loop() {
  // Handle serial commands
  handleSerialCommands();

  // Handle HTTP requests
  server.handleClient();
  
  // Handle BLE connection changes
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  // Generate and log sensor data at intervals
  unsigned long currentTime = millis();
  if (currentTime - lastLogTime >= LOG_INTERVAL) {
    
    // Read real sensor data
    SensorData data = {
      .timestamp = currentTime,
      .temperature = readTemperature(),     // Real temperature reading
      .humidity = readHumidity(),           // Real humidity reading  
      .batteryVoltage = readBatteryVoltage(), // Real battery reading
      .tamperState = readTamperState(),     // Real tamper detection
      .bootNumber = bootCount
    };
    
    // Log to storage
    logSensorData(data);
    
    // Broadcast via BLE
    broadcastBLEData(data);
    
    lastLogTime = currentTime;
  }
  
  delay(100);
} 