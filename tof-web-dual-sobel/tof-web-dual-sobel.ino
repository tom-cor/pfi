#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <vl53l8cx.h>
#include <Adafruit_NeoPixel.h>

// --- Configuration ---
const char* ssid = "test_espcam";
const char* password = "testing123";

// --- SPI Pin Definitions ---
#define SPI_MOSI 13
#define SPI_SCK  12
#define SPI_MISO 11

#define CS_PIN_1 10
#define CS_PIN_2 9
#define LPN_PIN  -1 

VL53L8CX sensor1(&SPI, CS_PIN_1, LPN_PIN);
VL53L8CX sensor2(&SPI, CS_PIN_2, LPN_PIN);

#define NEOPIXEL_PIN        48  
#define NUM_PIXELS           1  

Adafruit_NeoPixel statusLED(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- Global State ---
int currentZones = 64; 
bool pendingResolutionChange = false;
int newZonesTarget = 64;

VL53L8CX_ResultsData data1;
VL53L8CX_ResultsData data2;
bool hasData1 = false;
bool hasData2 = false;

// Global fault flags so the loop knows who is alive
uint8_t sensor1fault = 0;
uint8_t sensor2fault = 0;

// --- Embedded HTML/JS Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Dual ToF Directional Edge Detection</title>
  <style>
    body { font-family: Arial; background-color: #121212; color: white; text-align: center; margin: 0; padding: 20px; box-sizing: border-box; overflow-x: hidden; }
    #controls { margin-bottom: 20px; }
    button { padding: 10px 20px; font-size: 16px; cursor: pointer; background-color: #4CAF50; color: white; border: none; border-radius: 5px; }
    button:hover { background-color: #45a049; }
    
    .dashboard { display: flex; justify-content: center; gap: 20px; flex-wrap: wrap; width: 100%; }
    .sensor-container { display: flex; flex-direction: column; align-items: center; flex: 1 1 300px; max-width: 450px; }
    
    .grid { 
      display: grid; 
      gap: 3px; 
      width: 100%; 
      margin-top: 10px;
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
      font-size: 11px;
      box-sizing: border-box;
      transition: border 0.1s;
      border: 2px solid transparent; 
      overflow: hidden; 
      line-height: 1.1;
      padding: 1px;
    }
    .status { font-size: 9px; color: #ddd; margin-top: 2px; font-weight: normal; }
    .sobel { font-size: 11px; font-weight: bold; margin-top: 2px; }
    
    .edge-climb { border-color: #2196F3; } 
    .edge-drop  { border-color: #f44336; } 
  </style>
</head>
<body>
  <h2>VL53L8CX Directional Edge Detection</h2>
  <div id="controls">
    <button id="resBtn" onclick="toggleResolution()">Switch to 4x4 Mode</button>
  </div>
  
  <div class="dashboard">
    <div class="sensor-container">
      <h3>Sensor 1 (Left)</h3>
      <div id="grid1" class="grid"></div>
    </div>
    <div class="sensor-container">
      <h3>Sensor 2 (Right)</h3>
      <div id="grid2" class="grid"></div>
    </div>
  </div>
  
  <script>
    const grid1 = document.getElementById('grid1');
    const grid2 = document.getElementById('grid2');
    let cells1 = [];
    let cells2 = [];
    let currentRes = 0;

    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket = new WebSocket(gateway);
    
    function toggleResolution() {
      if (currentRes === 8) {
        websocket.send("RES:4");
        document.getElementById('resBtn').innerText = "Switching...";
      } else {
        websocket.send("RES:8");
        document.getElementById('resBtn').innerText = "Switching...";
      }
    }

    function initGrid(gridElement, cellArray, res) {
      gridElement.style.gridTemplateColumns = `repeat(${res}, 1fr)`;
      gridElement.innerHTML = '';
      cellArray.length = 0;
      for (let i = 0; i < res * res; i++) {
        let div = document.createElement('div');
        div.className = 'cell';
        gridElement.appendChild(div);
        cellArray.push(div);
      }
    }

    function updateGridData(cellArray, dataString) {
      if (!dataString) return;
      let pairs = dataString.split(',');
      let res = Math.sqrt(pairs.length);

      for (let i = 0; i < pairs.length; i++) {
        let y = Math.floor(i / res);
        let x = i % res;
        let flippedX = (res - 1) - x;
        let flippedIndex = (y * res) + flippedX;
        
        if (!cellArray[flippedIndex]) continue; 
        
        let targetCell = cellArray[flippedIndex];

        let parts = pairs[i].split(':');
        let dist = parseInt(parts[0]);
        let stat = parseInt(parts[1]);
        let sobelClass = parseInt(parts[2]);
        
        let hue = Math.min(240, 120 + (dist / 2000) * 120); 
        
        if (dist === 0 || dist > 4000 || (stat !== 5 && stat !== 9)) {
          targetCell.style.backgroundColor = '#333';
          targetCell.className = 'cell';
          targetCell.innerHTML = `- <br><span class="status">S:${stat}</span>`;
        } else {
          targetCell.style.backgroundColor = `hsl(${hue}, 100%, 35%)`;
          targetCell.innerHTML = `${dist} <br><span class="status">S:${stat}</span>`;
          
          if (sobelClass === 1) {
            targetCell.className = 'cell edge-climb';
            targetCell.innerHTML += `<br><span class="sobel" style="color:#2196F3;">C</span>`;
          } else if (sobelClass === 2) {
            targetCell.className = 'cell edge-drop';
            targetCell.innerHTML += `<br><span class="sobel" style="color:#f44336;">D</span>`;
          } else {
            targetCell.className = 'cell'; 
          }
        }
      }
    }

    websocket.onmessage = function(event) {
      let sensorData = event.data.split('|');
      if (sensorData.length !== 2) return;

      let pairs1 = sensorData[0].split(',');
      let newRes = Math.sqrt(pairs1.length); 
      
      if (newRes !== currentRes && (newRes === 4 || newRes === 8)) {
        currentRes = newRes;
        initGrid(grid1, cells1, currentRes);
        initGrid(grid2, cells2, currentRes);
        document.getElementById('resBtn').innerText = `Switch to ${currentRes === 8 ? '4x4' : '8x8'} Mode`;
      }

      updateGridData(cells1, sensorData[0]);
      updateGridData(cells2, sensorData[1]);
    };
  </script>
</body>
</html>
)rawliteral";

// --- Sobel Helper Functions ---

int getPaddedDistance(VL53L8CX_ResultsData* data, int x, int y, int grid_size) {
  if (x < 0) x = 0;
  else if (x >= grid_size) x = grid_size - 1;
  
  if (y < 0) y = 0;
  else if (y >= grid_size) y = grid_size - 1;
  
  return data->distance_mm[(y * grid_size) + x];
}

void applySobel(VL53L8CX_ResultsData* data, int* outputArray, int total_zones) {
  int grid_size = sqrt(total_zones); 
  memset(outputArray, 0, total_zones * sizeof(int)); 

  for (int y = 0; y < grid_size; y++) {
    for (int x = 0; x < grid_size; x++) {
      
      int top_left  = getPaddedDistance(data, x - 1, y - 1, grid_size);
      int top       = getPaddedDistance(data, x,     y - 1, grid_size);
      int top_right = getPaddedDistance(data, x + 1, y - 1, grid_size);
      int mid_left  = getPaddedDistance(data, x - 1, y,     grid_size);
      int mid_right = getPaddedDistance(data, x + 1, y,     grid_size);
      int bot_left  = getPaddedDistance(data, x - 1, y + 1, grid_size);
      int bot       = getPaddedDistance(data, x,     y + 1, grid_size);
      int bot_right = getPaddedDistance(data, x + 1, y + 1, grid_size);

      int gx = (-1 * top_left)  + (1 * top_right) + 
               (-2 * mid_left)  + (2 * mid_right) + 
               (-1 * bot_left)  + (1 * bot_right);

      int gy = (-1 * top_left)  + (-2 * top) + (-1 * top_right) + 
               ( 1 * bot_left)  + ( 2 * bot) + ( 1 * bot_right);

      int magnitude = abs(gx) + abs(gy);
      int edgeClass = 0; 

      if (magnitude > 1000) { 
        if (abs(gy) > abs(gx)) {
          if (gy > 0) {
            edgeClass = 2; // DROP
          } else {
            edgeClass = 1; // CLIMB
          }
        } else {
          edgeClass = 1; 
        }
      }

      outputArray[(y * grid_size) + x] = edgeClass;
    }
  }
}

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

  statusLED.begin();           
  statusLED.setBrightness(50); 
  statusLED.clear();
  statusLED.show();            

  pinMode(CS_PIN_1, OUTPUT);
  digitalWrite(CS_PIN_1, HIGH); 
  
  pinMode(CS_PIN_2, OUTPUT);
  digitalWrite(CS_PIN_2, HIGH);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1); 

  Serial.println("Initializing Sensors over SPI...");
  
  sensor1.begin();
  delay(500); 
  sensor2.begin();

  if (sensor1.init() != 0) {
    Serial.println("Sensor 1 init failed!");
    sensor1fault = 1;
    // Zero out the struct so it sends safe empty arrays if dead
    memset(&data1, 0, sizeof(VL53L8CX_ResultsData)); 
  } else {
    sensor1.set_resolution(VL53L8CX_RESOLUTION_8X8);
    sensor1.set_ranging_frequency_hz(15);
    sensor1.start_ranging();
  }

  delay(500);

  if (sensor2.init() != 0){
    Serial.println("Sensor 2 init failed!");
    sensor2fault = 1;
    memset(&data2, 0, sizeof(VL53L8CX_ResultsData)); 
  } else {
    sensor2.set_resolution(VL53L8CX_RESOLUTION_8X8);
    sensor2.set_ranging_frequency_hz(15);
    sensor2.start_ranging();
  }

  statusLED.setPixelColor(0, statusLED.Color(sensor1fault * 255, 0, sensor2fault * 255));
  statusLED.show();

  delay(500);

  // --- Wi-Fi Connection & AP Fallback ---
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  unsigned long startAttemptTime = millis();
  
  // Keep trying until connected OR 10 seconds have passed
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    statusLED.setPixelColor(0, statusLED.Color(sensor1fault * 255, 255, sensor2fault * 255));
    statusLED.show();
    delay(250);
    Serial.print(".");

    statusLED.setPixelColor(0, statusLED.Color(sensor1fault * 255, 0, sensor2fault * 255));
    statusLED.show();
    delay(250);
  }

  // Check if we successfully connected or if we timed out
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP Address: " + WiFi.localIP().toString());
    
    // Set to solid base fault color
    statusLED.setPixelColor(0, statusLED.Color(sensor1fault * 255, 0, sensor2fault * 255));
    statusLED.show();
  } else {
    Serial.println("\nWiFi Failed! Starting Access Point...");
    
    // Switch to AP mode and broadcast a network called "NavAid-Config"
    WiFi.mode(WIFI_AP);
    WiFi.softAP("NavAid-Config"); 
    // Note: To add a password, use WiFi.softAP("NavAid-Config", "yourpassword");
    
    Serial.println("AP IP Address: " + WiFi.softAPIP().toString()); // Usually defaults to 192.168.4.1

    // Flash white rapidly 3 times to visually signal AP mode entered
    for(int i = 0; i < 3; i++) {
       statusLED.setPixelColor(0, statusLED.Color(255, 255, 255));
       statusLED.show();
       delay(100);
       statusLED.setPixelColor(0, statusLED.Color(0, 0, 0));
       statusLED.show();
       delay(100);
    }
    
    // Return to the base fault diagnostic color
    statusLED.setPixelColor(0, statusLED.Color(sensor1fault * 255, 0, sensor2fault * 255));
    statusLED.show();
  }

  // --- Start Web Server (Works for both STA and AP mode) ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  if (pendingResolutionChange) {
    // Only attempt to change resolution on sensors that are actually alive
    if (!sensor1fault) sensor1.stop_ranging();
    if (!sensor2fault) sensor2.stop_ranging();
    
    int res = (newZonesTarget == 16) ? VL53L8CX_RESOLUTION_4X4 : VL53L8CX_RESOLUTION_8X8;
    
    if (!sensor1fault) {
      sensor1.set_resolution(res);
      sensor1.set_ranging_frequency_hz(15); 
      sensor1.start_ranging();
    }
    
    if (!sensor2fault) {
      sensor2.set_resolution(res);
      sensor2.set_ranging_frequency_hz(15);
      sensor2.start_ranging();
    }
    
    currentZones = newZonesTarget;
    hasData1 = false;
    hasData2 = false;
    pendingResolutionChange = false;
  }

  // Poll Sensor 1 if healthy. If faulted, artificially pass the gate.
  if (!sensor1fault && !hasData1) {
    uint8_t ready1 = 0;
    sensor1.check_data_ready(&ready1);
    if (ready1) {
      if (sensor1.get_ranging_data(&data1) == 0) hasData1 = true;
    }
  } else if (sensor1fault) {
    hasData1 = true;
  }

  // Poll Sensor 2 if healthy. If faulted, artificially pass the gate.
  if (!sensor2fault && !hasData2) {
    uint8_t ready2 = 0;
    sensor2.check_data_ready(&ready2);
    if (ready2) {
      if (sensor2.get_ranging_data(&data2) == 0) hasData2 = true;
    }
  } else if (sensor2fault) {
    hasData2 = true;
  }
  
  if (hasData1 && hasData2) {
    
    // Flood prevention: If both are dead, throttle the loop so the ESP doesn't crash the browser
    if (sensor1fault && sensor2fault) {
      delay(100); 
    }

    int sobel1[64] = {0};
    int sobel2[64] = {0};
    
    applySobel(&data1, sobel1, currentZones);
    applySobel(&data2, sobel2, currentZones);

    String payload = "";
    
    // Package Sensor 1
    for (int i = 0; i < currentZones; i++) {
      payload += String(data1.distance_mm[i]) + ":" + String(data1.target_status[i]) + ":" + String(sobel1[i]);
      if (i < currentZones - 1) payload += ",";
    }
    
    payload += "|"; 
    
    // Package Sensor 2
    for (int i = 0; i < currentZones; i++) {
      payload += String(data2.distance_mm[i]) + ":" + String(data2.target_status[i]) + ":" + String(sobel2[i]);
      if (i < currentZones - 1) payload += ",";
    }
    
    ws.textAll(payload);
    
    hasData1 = false;
    hasData2 = false;
  }
  
  ws.cleanupClients();
}