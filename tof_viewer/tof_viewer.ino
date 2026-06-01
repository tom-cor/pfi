#include <Arduino.h>
#include <SPI.h>
#include <vl53l8cx.h>

// --- ESP32-S3 SPI Pins ---
#define SPI_SCK  12
#define SPI_MISO 13
#define SPI_MOSI 11
#define SPI_CS   10

// --- IMU & Version Placeholders ---
// The Python script expects these. Update these variables with 
// real data once you connect your IMU.
float quatW = 1.000000;
float quatX = 0.000000;
float quatY = 0.000000;
float quatZ = 0.000000;
#define VERSION "1.0"

// Instantiate the sensor object
VL53L8CX sensor_vl53l8cx(&SPI, SPI_CS, -1);

void setup() {
  Serial.begin(115200);
  // Give the Serial port a moment to open
  delay(2000); 

  // Initialize SPI
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  sensor_vl53l8cx.begin();
  
  // Hardware check
  int status = sensor_vl53l8cx.init();
  if (status != 0) {
    // Print a JSON formatted error just in case the Python script is listening
    Serial.print("{\"error\":\"Hardware not found\", \"code\":");
    Serial.print(status);
    Serial.println("}");
    while(1); // Freeze on error
  }
  
  // Max resolution and frame rate
  sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_8X8);
  sensor_vl53l8cx.set_ranging_frequency_hz(15);
  sensor_vl53l8cx.start_ranging();
}

void loop() {
  // Use the specific data struct for our library
  VL53L8CX_ResultsData data;
  uint8_t data_ready = 0;

  // Poll for new ToF data
  sensor_vl53l8cx.check_data_ready(&data_ready);

  if (data_ready) {
    // Fetch the 64 zones
    sensor_vl53l8cx.get_ranging_data(&data);

    // Output JSON with distance, status, and quaternion data
    Serial.print("{\"distances\":[");
    for (int i = 0; i < 64; i++) {
      Serial.print(data.distance_mm[i]);
      if (i < 63) Serial.print(",");
    }

    Serial.print("],\"status\":[");
    for (int i = 0; i < 64; i++) {
      // The library stores the target status in the same struct
      Serial.print(data.target_status[i]);
      if (i < 63) Serial.print(",");
    }

    // Add quaternion (wxyz format) with 6 decimal places for accuracy
    Serial.print("],\"quat\":[");
    Serial.print(quatW, 6); Serial.print(",");
    Serial.print(quatX, 6); Serial.print(",");
    Serial.print(quatY, 6); Serial.print(",");
    Serial.print(quatZ, 6);
    
    // Add version and close the JSON payload
    Serial.print("],\"v\":\"");
    Serial.print(VERSION);
    Serial.println("\"}");
  }
  
  // A tiny delay to prevent overwhelming the ESP32
  delay(5); 
}
