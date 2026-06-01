#include <Arduino.h>
#include <Wire.h>
#include <vl53l8cx.h> // Corrected header

// Instantiate the sensor object. 
// (-1) means we are handling the LPn pin manually.
VL53L8CX sensor_vl53l8cx(&Wire, -1);

void setup() {
  Serial.begin(115200);
  delay(2000); 
  Serial.println("Starting I2C Test...");

  // Initialize I2C with default ESP32-S3 pins: SDA = 8, SCL = 9
  Wire.begin(8, 9);
  Wire.setClock(100000); // Safe 100kHz speed

  // 1. Prepare the I2C software interface
  sensor_vl53l8cx.begin();
  
  // 2. THE REAL HARDWARE CHECK: Boot up the sensor
  Serial.println("Pinging VL53L8CX hardware... (This takes a moment)");
  
  // ---> CHANGED: init_sensor() is now init() <---
  int status = sensor_vl53l8cx.init();
  
  if (status != 0) {
    Serial.print("CRITICAL FAILURE! Hardware not found. Error code: ");
    Serial.println(status);
    while(1); // Freeze here
  }
  
  Serial.println("SUCCESS! Sensor is alive and initialized.");
  
  // Set resolution and start ranging
  sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_4X4);
  sensor_vl53l8cx.start_ranging();
}

void loop() {
  VL53L8CX_ResultsData data;
  uint8_t data_ready = 0;

  // Heartbeat so we know the ESP32 hasn't crashed
  static unsigned long last_tick = 0;
  if (millis() - last_tick > 1000) {
    Serial.print("."); 
    last_tick = millis();
  }

  sensor_vl53l8cx.check_data_ready(&data_ready);

  if (data_ready) {
    sensor_vl53l8cx.get_ranging_data(&data);

    Serial.println("\n--- 4x4 I2C Frame ---");
    
    for (int i = 0; i < 16; i++) {
      Serial.print(data.distance_mm[i]);
      Serial.print("\t");
      
      if ((i + 1) % 4 == 0) {
        Serial.println();
      }
    }
  }
  
  delay(10); 
}