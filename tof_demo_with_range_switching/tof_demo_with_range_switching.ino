#include <Arduino.h>
#include <SPI.h>
#include <vl53l8cx.h>

// --- ESP32-S3 SPI & Switch Pins ---
#define SPI_SCK  12
#define SPI_MISO 13
#define SPI_MOSI 11
#define SPI_CS   10
#define PIN_RES_SWITCH 4 // The physical toggle switch pin

// --- IMU & Version Placeholders ---
float quatW = 1.000000;
float quatX = 0.000000;
float quatY = 0.000000;
float quatZ = 0.000000;
#define VERSION "1.0"

VL53L8CX sensor_vl53l8cx(&SPI, SPI_CS, -1);

// State tracking variables
int last_pin_state = -1;
int current_zones = 64; // Will dynamically switch to 16 or 64

void setup() {
  Serial.begin(115200);
  delay(2000); 

  // Initialize the switch pin with an internal pull-up
  pinMode(PIN_RES_SWITCH, INPUT_PULLUP);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  sensor_vl53l8cx.begin();
  
  int status = sensor_vl53l8cx.init();
  if (status != 0) {
    Serial.print("{\"error\":\"Hardware not found\", \"code\":");
    Serial.print(status);
    Serial.println("}");
    while(1); 
  }
  
  // Set initial resolution to 8x8 to match our default state
  sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_8X8);
  sensor_vl53l8cx.set_ranging_frequency_hz(15);
  sensor_vl53l8cx.start_ranging();
}

void loop() {
  // --- 1. Check for Resolution Switch Change ---
  int current_pin_state = digitalRead(PIN_RES_SWITCH);
  
  if (current_pin_state != last_pin_state) {
    // Stop the laser to safely change settings
    sensor_vl53l8cx.stop_ranging();
    
    if (current_pin_state == LOW) {
      // Switch closed (connected to GND) -> 8x8 Mode
      sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_8X8);
      current_zones = 64;
    } else {
      // Switch open (floating HIGH) -> 4x4 Mode
      sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_4X4);
      current_zones = 16;
    }
    
    // Restart the laser
    sensor_vl53l8cx.start_ranging();
    last_pin_state = current_pin_state;
  }

  // --- 2. Fetch and Format Data ---
  VL53L8CX_ResultsData data;
  uint8_t data_ready = 0;

  sensor_vl53l8cx.check_data_ready(&data_ready);

  if (data_ready) {
    sensor_vl53l8cx.get_ranging_data(&data);

    Serial.print("{\"distances\":[");
    for (int i = 0; i < current_zones; i++) {
      Serial.print(data.distance_mm[i]);
      if (i < current_zones - 1) Serial.print(",");
    }

    Serial.print("],\"status\":[");
    for (int i = 0; i < current_zones; i++) {
      Serial.print(data.target_status[i]);
      if (i < current_zones - 1) Serial.print(",");
    }

    Serial.print("],\"quat\":[");
    Serial.print(quatW, 6); Serial.print(",");
    Serial.print(quatX, 6); Serial.print(",");
    Serial.print(quatY, 6); Serial.print(",");
    Serial.print(quatZ, 6);
    
    Serial.print("],\"v\":\"");
    Serial.print(VERSION);
    Serial.println("\"}");
  }
  
  delay(5); 
}