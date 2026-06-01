#include <Arduino.h>
#include <SPI.h>
#include <vl53l8cx.h>

#define SPI_SCK  12
#define SPI_MISO 13
#define SPI_MOSI 11
#define SPI_CS   10

VL53L8CX sensor_vl53l8cx(&SPI, SPI_CS, -1);

void setup() {
  Serial.begin(115200);
  delay(2000); 

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  sensor_vl53l8cx.begin();
  
  int status = sensor_vl53l8cx.init();
  if (status != 0) {
    Serial.print("CRITICAL FAILURE! Hardware not found. Error code: ");
    Serial.println(status);
    while(1); 
  }
  
  sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_8X8);
  sensor_vl53l8cx.set_ranging_frequency_hz(15);
  sensor_vl53l8cx.start_ranging();

  // Clear the terminal screen entirely before we start the loop
  Serial.print("\033[2J"); 
}

void loop() {
  VL53L8CX_ResultsData data;
  uint8_t data_ready = 0;

  sensor_vl53l8cx.check_data_ready(&data_ready);

  if (data_ready) {
    sensor_vl53l8cx.get_ranging_data(&data);

    // ANSI code: Teleport cursor to the top-left (Home)
    Serial.print("\033[H"); 
    Serial.println("--- 8x8 Live Radar (mm) ---");
    
    for (int i = 0; i < 64; i++) {
      
      // Force every number to be 4 characters wide to prevent "ghost" digits
      char buffer[10];
      sprintf(buffer, "%4d\t", data.distance_mm[i]);
      Serial.print(buffer);
      
      if ((i + 1) % 8 == 0) {
        Serial.println();
      }
    }
  }
  
  delay(5); 
}