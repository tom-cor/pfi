#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <vl53l8cx.h> // Updated header

// --- Configuration ---
const char* ssid = "test_espcam";
const char* password = "testing123";

// --- SPI Pin Definitions ---
#define SPI_MOSI 9
#define SPI_SCK  10
#define SPI_MISO 11

// Unique Chip Select pins for each sensor
#define CS_PIN_1 12
#define CS_PIN_2 13
#define LPN_PIN  -1  

// Instantiate two sensor objects
VL53L8CX sensor1(&SPI, CS_PIN_1, LPN_PIN);
VL53L8CX sensor2(&SPI, CS_PIN_2, LPN_PIN);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- Global State ---
int currentZones = 64; 
bool pendingResolutionChange = false;
int newZonesTarget = 64;

// Buffers to hold data until both are ready
VL53L8CX_ResultsData data1;
VL53L8CX_ResultsData data2;
bool hasData1 = false;
bool hasData2 = false;

// --- Embedded HTML/JS Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Dual ESP32 ToF Heatmap</title>
  <style>
    body { font-family: Arial; background-color: #121212; color: white; text-align: center; margin: 0; padding: 20px; }
    #controls { margin-bottom: 20px; }
    button { padding: 10px 20px; font-size: 16px; cursor: pointer; background-color: #4CAF50; color: white; border: none; border-radius: 5px; }
    button:hover { background-color: #45a049; }
    
    .dashboard { display: flex; justify-content: center; gap: 40px; flex-wrap: wrap; }
    .sensor-container { display: flex; flex-direction: column; align-items: center; }
    
    .grid { 
      display: grid; 
      gap: 5px; 
      width: 400px; 
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
      font-size: 12px;
    }
    .status { font-size: 10px; color: #ddd; margin-top: 4px; font-weight: normal; }
  </style>
</head>
<body>
  <h2>Dual VL53L8CX Live Heatmap</h2>
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
      for (let i = 0; i < pairs.length; i++) {
        if(!cells1[i]) break; // Safety check during transition
        let parts = pairs[i].split(':');
        let dist = parseInt(parts[0]);
        let stat = parseInt(parts[1]);
        
        let hue = Math.max(0, 120 - (dist / 2000) * 120); 
        
        if (dist === 0 || dist > 4000 || (stat !== 5 && stat !== 9)) {
          cellArray[i].style.backgroundColor = '#333';
          cellArray[i].innerHTML = `- <br><span class="status">S:${stat}</span>`;
        } else {
          cellArray[i].style.backgroundColor = `hsl(${hue}, 100%, 35%)`;
          cellArray[i].innerHTML = `${dist} <br><span class="status">S:${stat}</span>`;
        }
      }
    }

    websocket.onmessage = function(event) {
      // Split payload: "Sensor1Data|Sensor2Data"
      let sensorData = event.data.split('|');
      if (sensorData.length !== 2) return;

      let pairs1 = sensorData[0].split(',');
      let newRes = Math.sqrt(pairs1.length); 
      
      // Rebuild grids if resolution changed
      if (newRes !== currentRes && (newRes === 4 || newRes === 8)) {
        currentRes = newRes;
        initGrid(grid1, cells1, currentRes);
        initGrid(grid2, cells2, currentRes);
        document.getElementById('resBtn').innerText = `Switch to ${currentRes === 8 ? '4x4' : '8x8'} Mode`;
      }

      // Update both grids
      updateGridData(cells1, sensorData[0]);
      updateGridData(cells2, sensorData[1]);
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

  // --- THE FIX: Force both sensors OFF the SPI bus immediately ---
  pinMode(CS_PIN_1, OUTPUT);
  digitalWrite(CS_PIN_1, HIGH); 
  
  pinMode(CS_PIN_2, OUTPUT);
  digitalWrite(CS_PIN_2, HIGH);
  // ---------------------------------------------------------------

  // Initialize the SPI bus. Use -1 for the default CS since we manage them manually now.
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1); 

  Serial.println("Initializing Sensors over SPI...");
  
  // Now initialize Sensor 1 (Sensor 2 is safely ignoring the bus)
  sensor1.begin();
  if (sensor1.init() != 0) {
    Serial.println("Sensor 1 init failed!");
  } else {
    Serial.println("Sensor 1 initialized.");
    sensor1.set_resolution(VL53L8CX_RESOLUTION_8X8);
    sensor1.set_ranging_frequency_hz(15);
    sensor1.start_ranging();
  }

  delay(1000);

  // Next, initialize Sensor 2
  sensor2.begin();
  if (sensor2.init() != 0) {
    Serial.println("Sensor 2 init failed!");
  } else {
    Serial.println("Sensor 2 initialized.");
    sensor2.set_resolution(VL53L8CX_RESOLUTION_8X8);
    sensor2.set_ranging_frequency_hz(15);
    sensor2.start_ranging();
  }

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
  // Handle Resolution Switching Safely
  if (pendingResolutionChange) {
    sensor1.stop_ranging();
    sensor2.stop_ranging();
    
    int res = (newZonesTarget == 16) ? VL53L8CX_RESOLUTION_4X4 : VL53L8CX_RESOLUTION_8X8;
    
    sensor1.set_resolution(res);
    sensor1.set_ranging_frequency_hz(15); 
    sensor2.set_resolution(res);
    sensor2.set_ranging_frequency_hz(15);
    
    sensor1.start_ranging();
    sensor2.start_ranging();
    
    currentZones = newZonesTarget;
    hasData1 = false;
    hasData2 = false;
    pendingResolutionChange = false;
  }

  // Polling Sensor 1
  if (!hasData1) {
    uint8_t ready1 = 0;
    sensor1.check_data_ready(&ready1);
    if (ready1) {
      if (sensor1.get_ranging_data(&data1) == 0) hasData1 = true;
    }
  }

  // Polling Sensor 2
  if (!hasData2) {
    uint8_t ready2 = 0;
    sensor2.check_data_ready(&ready2);
    if (ready2) {
      if (sensor2.get_ranging_data(&data2) == 0) hasData2 = true;
    }
  }
  
  // Package and send when BOTH frames are acquired
  if (hasData1 && hasData2) {
    String payload = "";
    
    // Process Sensor 1
    for (int i = 0; i < currentZones; i++) {
      payload += String(data1.distance_mm[i]) + ":" + String(data1.target_status[i]);
      if (i < currentZones - 1) payload += ",";
    }
    
    payload += "|"; // Separator
    
    // Process Sensor 2
    for (int i = 0; i < currentZones; i++) {
      payload += String(data2.distance_mm[i]) + ":" + String(data2.target_status[i]);
      if (i < currentZones - 1) payload += ",";
    }
    
    ws.textAll(payload);
    
    // Reset flags for the next frame
    hasData1 = false;
    hasData2 = false;
  }
  
  ws.cleanupClients();
}