#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <vl53l8cx.h>

// --- Configuration ---
const char* ssid = "test_espcam";
const char* password = "testing123";

// --- SPI Pin Definitions ---
#define SPI_MOSI 9
#define SPI_SCK  10
#define SPI_MISO 11
#define CS_PIN   12
#define LPN_PIN  -1  

VL53L8CX sensor_vl53l8cx(&SPI, CS_PIN, LPN_PIN);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- Global State for Resolution ---
int currentZones = 64; // Defaults to 8x8
bool pendingResolutionChange = false;
int newZonesTarget = 64;

// --- Embedded HTML/JS Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 ToF Heatmap</title>
  <style>
    body { font-family: Arial; background-color: #121212; color: white; text-align: center; }
    #controls { margin: 20px; }
    button { padding: 10px 20px; font-size: 16px; cursor: pointer; background-color: #4CAF50; color: white; border: none; border-radius: 5px; }
    button:hover { background-color: #45a049; }
    #grid { 
      display: grid; 
      gap: 5px; 
      max-width: 600px; 
      margin: 20px auto; 
    }
    .cell { 
      aspect-ratio: 1 / 1; 
      display: flex; 
      flex-direction: column;
      align-items: center; 
      justify-content: center; 
      background-color: #333; 
      color: #fff; 
      font-weight: bold; 
      border-radius: 4px;
      font-size: 14px;
    }
    .status { font-size: 10px; color: #ddd; margin-top: 4px; font-weight: normal; }
  </style>
</head>
<body>
  <h2>VL53L8CX Live Heatmap (SPI)</h2>
  <div id="controls">
    <button id="resBtn" onclick="toggleResolution()">Switch to 4x4 Mode</button>
  </div>
  <div id="grid"></div>
  
  <script>
    const grid = document.getElementById('grid');
    let cells = [];
    let currentRes = 0; // Will be set dynamically based on data

    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket = new WebSocket(gateway);
    
    function toggleResolution() {
      // Send command to ESP32
      if (currentRes === 8) {
        websocket.send("RES:4");
        document.getElementById('resBtn').innerText = "Switching...";
      } else {
        websocket.send("RES:8");
        document.getElementById('resBtn').innerText = "Switching...";
      }
    }

    websocket.onmessage = function(event) {
      let pairs = event.data.split(',');
      let newRes = Math.sqrt(pairs.length); 
      
      // Rebuild grid if resolution changed
      if (newRes !== currentRes && (newRes === 4 || newRes === 8)) {
        currentRes = newRes;
        grid.style.gridTemplateColumns = `repeat(${currentRes}, 1fr)`;
        grid.innerHTML = '';
        cells = [];
        for (let i = 0; i < currentRes * currentRes; i++) {
          let div = document.createElement('div');
          div.className = 'cell';
          grid.appendChild(div);
          cells.push(div);
        }
        document.getElementById('resBtn').innerText = `Switch to ${currentRes === 8 ? '4x4' : '8x8'} Mode`;
      }

      // Parse data and update UI
      for (let i = 0; i < pairs.length; i++) {
        let parts = pairs[i].split(':');
        let dist = parseInt(parts[0]);
        let stat = parseInt(parts[1]);
        
        let hue = Math.max(0, 120 - (dist / 2000) * 120); 
        
        // Status 5 and 9 are generally considered "Good" readings by the ST API
        if (dist === 0 || dist > 4000 || (stat !== 5 && stat !== 9)) {
          cells[i].style.backgroundColor = '#333';
          cells[i].innerHTML = `- <br><span class="status">Stat: ${stat}</span>`;
        } else {
          cells[i].style.backgroundColor = `hsl(${hue}, 100%, 35%)`;
          cells[i].innerHTML = `${dist} <br><span class="status">Stat: ${stat}</span>`;
        }
      }
    };
  </script>
</body>
</html>
)rawliteral";

// --- WebSocket Event Handler ---
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String message = (char*)data;
      
      if (message == "RES:4") {
        newZonesTarget = 16;
        pendingResolutionChange = true;
      } 
      else if (message == "RES:8") {
        newZonesTarget = 64;
        pendingResolutionChange = true;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS_PIN);
  delay(2000);

  Serial.println("Initializing VL53L8CX over SPI...");
  sensor_vl53l8cx.begin();
  delay(2000);
  
  if (sensor_vl53l8cx.init() != 0) {
    Serial.println("Sensor init failed! Check wiring and ensure I2C_SPI_N is tied to GND.");
    while (1);
  }
  
  sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_8X8);
  sensor_vl53l8cx.set_ranging_frequency_hz(15);
  sensor_vl53l8cx.start_ranging();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nIP Address: " + WiFi.localIP().toString());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  // Handle Resolution Switching Safely Outside Interrupts
  if (pendingResolutionChange) {
    sensor_vl53l8cx.stop_ranging();
    
    if (newZonesTarget == 16) {
      sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_4X4);
      // 4x4 mode can support higher framerates, up to 60Hz. Keeping at 15Hz for stability.
      sensor_vl53l8cx.set_ranging_frequency_hz(15); 
    } else {
      sensor_vl53l8cx.set_resolution(VL53L8CX_RESOLUTION_8X8);
      sensor_vl53l8cx.set_ranging_frequency_hz(15);
    }
    
    sensor_vl53l8cx.start_ranging();
    currentZones = newZonesTarget;
    pendingResolutionChange = false;
  }

  // Normal Ranging Routine
  VL53L8CX_ResultsData measurementData;
  uint8_t newDataReady = 0;
  
  sensor_vl53l8cx.check_data_ready(&newDataReady);
  
  if (newDataReady) {
    if (sensor_vl53l8cx.get_ranging_data(&measurementData) == 0) {
      
      String payload = "";
      for (int i = 0; i < currentZones; i++) {
        payload += String(measurementData.distance_mm[i]);
        payload += ":";
        payload += String(measurementData.target_status[i]);
        
        if (i < currentZones - 1) payload += ",";
      }
      
      ws.textAll(payload);
    }
  }
  
  ws.cleanupClients();
}