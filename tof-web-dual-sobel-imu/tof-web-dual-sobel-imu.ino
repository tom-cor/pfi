#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <Wire.h>
#include <vl53l8cx.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO08x.h>

// --- Configuration ---
const char* ssid = "test_espcam";
const char* password = "testing123";

// --- SPI Pin Definitions (ToF) ---
#define SPI_MOSI 13
#define SPI_SCK  12
#define SPI_MISO 11

#define CS_PIN_1 10
#define CS_PIN_2 9
#define LPN_PIN  -1 

VL53L8CX sensor1(&SPI, CS_PIN_1, LPN_PIN);
VL53L8CX sensor2(&SPI, CS_PIN_2, LPN_PIN);

// --- I2C Pin Definitions (IMU) ---
#define I2C_SDA 4
#define I2C_SCL 5
Adafruit_BNO08x bno08x(-1); 
sh2_SensorValue_t sensorValue;

// --- NeoPixel ---
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

uint8_t sensor1fault = 0;
uint8_t sensor2fault = 0;

// IMU Global State & Safety Gate
bool imuReady = false;
float currentRoll = 0.0;
float currentPitch = 0.0;
float currentYaw = 0.0;

// --- Embedded HTML/JS Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Dual ToF + IMU Dashboard</title>
  <style>
    body { font-family: Arial; background-color: #121212; color: white; text-align: center; margin: 0; padding: 20px; box-sizing: border-box; overflow-x: hidden; }
    #controls { margin-bottom: 20px; display: flex; justify-content: center; gap: 15px; }
    button { padding: 10px 20px; font-size: 14px; cursor: pointer; background-color: #4CAF50; color: white; border: none; border-radius: 5px; font-weight: bold; transition: background-color 0.2s; }
    button:hover { background-color: #45a049; }
    
    .imu-banner { background-color: #222; padding: 15px; border-radius: 8px; margin-bottom: 20px; font-size: 18px; font-weight: bold; border: 1px solid #444; }
    .imu-val { color: #4CAF50; margin-right: 15px; }

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
      font-size: 10px;
      box-sizing: border-box;
      transition: border 0.1s;
      border: 2px solid transparent; 
      overflow: hidden; 
      line-height: 1.1;
      padding: 1px;
    }
    .status { font-size: 8px; color: #ddd; margin-top: 2px; font-weight: normal; }
    .sobel { font-size: 10px; font-weight: bold; margin-top: 2px; }
    
    .edge-climb { border-color: #2196F3; } 
    .edge-drop  { border-color: #f44336; } 
  </style>
</head>
<body>
  <h2>Navigation Aid Dashboard</h2>
  
  <div class="imu-banner">
    Roll: <span class="imu-val" id="roll">0.0&deg;</span>
    Pitch: <span class="imu-val" id="pitch">0.0&deg;</span>
    Yaw: <span class="imu-val" id="yaw">0.0&deg;</span>
  </div>

  <div id="controls">
    <button id="resBtn" onclick="toggleResolution()">Switch to 4x4 Mode</button>
    <button id="decompBtn" onclick="toggleDecomp()">Enable Roll Decomposition</button>
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
    
    let isDecompEnabled = false;

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
    
    function toggleDecomp() {
      isDecompEnabled = !isDecompEnabled;
      let btn = document.getElementById('decompBtn');
      if (isDecompEnabled) {
        btn.innerText = "Disable Roll Decomposition";
        btn.style.backgroundColor = "#2196F3"; // Highlight Blue when active
      } else {
        btn.innerText = "Enable Roll Decomposition";
        btn.style.backgroundColor = "#4CAF50"; // Return to Green
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

    // Now accepts isLeftSensor boolean and the current roll angle
    function updateGridData(cellArray, dataString, isLeftSensor, rollAngle) {
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
          
          // --- MATH: Apply Vector Decomposition ---
          let displayStr = `${dist}`;
          if (isDecompEnabled && isLeftSensor) {
            
            // Apply the -45 degree physical mounting offset
            let effectiveRoll = rollAngle - 45.0; 
            
            // Convert to radians and do the trig
            let rollRad = effectiveRoll * (Math.PI / 180);
            let compX = Math.round(dist * Math.cos(rollRad));
            let compY = Math.round(dist * Math.sin(rollRad));
            
            displayStr = `X:${compX}<br>Y:${compY}`;
          }
          // ----------------------------------------

          targetCell.innerHTML = `${displayStr} <br><span class="status">S:${stat}</span>`;
          
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
      let currentRollDeg = 0;
      
      // Extract IMU data
      if (sensorData.length === 3) {
        let imuParts = sensorData[2].split(',');
        let rawRoll = parseFloat(imuParts[0]);
        
        // --- IMU ORIENTATION CORRECTION ---
        // Neutral reads -90, and the physical axis is inverted.
        currentRollDeg = -(rawRoll + 90.0);
        // ----------------------------------

        // Update the dashboard banner with the corrected roll
        document.getElementById('roll').innerText = currentRollDeg.toFixed(1) + '°';
        document.getElementById('pitch').innerText = parseFloat(imuParts[1]).toFixed(1) + '°';
        document.getElementById('yaw').innerText = parseFloat(imuParts[2]).toFixed(1) + '°';
      }

      let pairs1 = sensorData[0].split(',');
      let newRes = Math.sqrt(pairs1.length); 
      
      if (newRes !== currentRes && (newRes === 4 || newRes === 8)) {
        currentRes = newRes;
        initGrid(grid1, cells1, currentRes);
        initGrid(grid2, cells2, currentRes);
        document.getElementById('resBtn').innerText = `Switch to ${currentRes === 8 ? '4x4' : '8x8'} Mode`;
      }

      // Pass the specific flags and the CORRECTED roll angle down to the UI renderer
      updateGridData(cells1, sensorData[0], true, currentRollDeg);
      updateGridData(cells2, sensorData[1], false, currentRollDeg);
    };
  </script>
</body>
</html>
)rawliteral";

// --- Helper Functions ---

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

      int gx = (-1 * top_left)  + (1 * top_right) + (-2 * mid_left)  + (2 * mid_right) + (-1 * bot_left)  + (1 * bot_right);
      int gy = (-1 * top_left)  + (-2 * top) + (-1 * top_right) + ( 1 * bot_left)  + ( 2 * bot) + ( 1 * bot_right);

      int magnitude = abs(gx) + abs(gy);
      int edgeClass = 0; 

      if (magnitude > 1000) { 
        if (abs(gy) > abs(gx)) {
          if (gy > 0) edgeClass = 2; // DROP
          else edgeClass = 1; // CLIMB
        } else {
          edgeClass = 1; 
        }
      }

      outputArray[(y * grid_size) + x] = edgeClass;
    }
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String message = (char*)data;
      if (message == "RES:4") { newZonesTarget = 16; pendingResolutionChange = true; } 
      else if (message == "RES:8") { newZonesTarget = 64; pendingResolutionChange = true; }
    }
  }
}

void setup() {
  Serial.begin(115200);

  statusLED.begin();           
  statusLED.setBrightness(50); 
  statusLED.clear();
  statusLED.show();            

  // --- Initialize I2C and BNO085 ---
  // Explicitly start I2C on your custom pins
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // CRITICAL: The BNO085 processor takes time to boot up. 
  // Give it a moment before hammering it with I2C requests.
  delay(250); 

  Serial.println("Looking for BNO085...");

  // Try the default address (0x4A) and explicitly pass the 'Wire' object
  if (!bno08x.begin_I2C(0x4A, &Wire)) {
    Serial.println("Failed on 0x4A. Trying alternate address 0x4B...");
    
    // Try the alternate address (0x4B)
    if (!bno08x.begin_I2C(0x4B, &Wire)) {
      Serial.println("BNO085 IMU not found on either address! (Check wiring)");
      imuReady = false; 
    } else {
      Serial.println("BNO085 IMU initialized on 0x4B.");
      bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 20000); 
      imuReady = true;  
    }
  } else {
    Serial.println("BNO085 IMU initialized on 0x4A.");
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 20000); 
    imuReady = true;  
  }

  // --- Initialize SPI and ToFs ---
  pinMode(CS_PIN_1, OUTPUT); digitalWrite(CS_PIN_1, HIGH); 
  pinMode(CS_PIN_2, OUTPUT); digitalWrite(CS_PIN_2, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1); 

  Serial.println("Initializing Sensors over SPI...");
  
  sensor1.begin(); delay(500); sensor2.begin();

  if (sensor1.init() != 0) {
    Serial.println("Sensor 1 init failed!");
    sensor1fault = 1;
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

  // --- Wi-Fi Connection & AP Fallback ---
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    statusLED.setPixelColor(0, statusLED.Color(sensor1fault * 255, 255, sensor2fault * 255));
    statusLED.show();
    delay(250);
    Serial.print(".");
    statusLED.setPixelColor(0, statusLED.Color(sensor1fault * 255, 0, sensor2fault * 255));
    statusLED.show();
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP Address: " + WiFi.localIP().toString());
    statusLED.setPixelColor(0, statusLED.Color(sensor1fault * 255, 0, sensor2fault * 255));
    statusLED.show();
  } else {
    Serial.println("\nWiFi Failed! Starting Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("NavAid-Config"); 
    Serial.println("AP IP Address: " + WiFi.softAPIP().toString()); 
    for(int i = 0; i < 3; i++) {
       statusLED.setPixelColor(0, statusLED.Color(255, 255, 255)); statusLED.show(); delay(100);
       statusLED.setPixelColor(0, statusLED.Color(0, 0, 0)); statusLED.show(); delay(100);
    }
    statusLED.setPixelColor(0, statusLED.Color(sensor1fault * 255, 0, sensor2fault * 255));
    statusLED.show();
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  // 1. Poll the IMU continuously (Gated by imuReady)
  if (imuReady) {
    if (bno08x.wasReset()) {
      bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 20000);
    }
    
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
        float qr = sensorValue.un.arvrStabilizedRV.real;
        float qi = sensorValue.un.arvrStabilizedRV.i;
        float qj = sensorValue.un.arvrStabilizedRV.j;
        float qk = sensorValue.un.arvrStabilizedRV.k;

        float sqr = sq(qr);
        float sqi = sq(qi);
        float sqj = sq(qj);
        float sqk = sq(qk);

        currentYaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
        currentPitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
        currentRoll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;
      }
    }
  }

  // 2. Handle Resolution Changes
  if (pendingResolutionChange) {
    if (!sensor1fault) sensor1.stop_ranging();
    if (!sensor2fault) sensor2.stop_ranging();
    int res = (newZonesTarget == 16) ? VL53L8CX_RESOLUTION_4X4 : VL53L8CX_RESOLUTION_8X8;
    if (!sensor1fault) { sensor1.set_resolution(res); sensor1.set_ranging_frequency_hz(15); sensor1.start_ranging(); }
    if (!sensor2fault) { sensor2.set_resolution(res); sensor2.set_ranging_frequency_hz(15); sensor2.start_ranging(); }
    currentZones = newZonesTarget;
    hasData1 = false; hasData2 = false;
    pendingResolutionChange = false;
  }

  // 3. Poll ToF Sensors
  if (!sensor1fault && !hasData1) {
    uint8_t ready1 = 0;
    sensor1.check_data_ready(&ready1);
    if (ready1 && sensor1.get_ranging_data(&data1) == 0) hasData1 = true;
  } else if (sensor1fault) { hasData1 = true; }

  if (!sensor2fault && !hasData2) {
    uint8_t ready2 = 0;
    sensor2.check_data_ready(&ready2);
    if (ready2 && sensor2.get_ranging_data(&data2) == 0) hasData2 = true;
  } else if (sensor2fault) { hasData2 = true; }
  
  // 4. Send WebSocket Data when ToF frames are ready
  if (hasData1 && hasData2) {
    if (sensor1fault && sensor2fault) delay(100); 

    int sobel1[64] = {0}; int sobel2[64] = {0};
    applySobel(&data1, sobel1, currentZones);
    applySobel(&data2, sobel2, currentZones);

    String payload = "";
    
    for (int i = 0; i < currentZones; i++) {
      payload += String(data1.distance_mm[i]) + ":" + String(data1.target_status[i]) + ":" + String(sobel1[i]);
      if (i < currentZones - 1) payload += ",";
    }
    payload += "|"; 
    
    for (int i = 0; i < currentZones; i++) {
      payload += String(data2.distance_mm[i]) + ":" + String(data2.target_status[i]) + ":" + String(sobel2[i]);
      if (i < currentZones - 1) payload += ",";
    }
    
    // Append the IMU angles to the payload
    payload += "|";
    payload += String(currentRoll, 1) + "," + String(currentPitch, 1) + "," + String(currentYaw, 1);
    
    ws.textAll(payload);
    
    hasData1 = false;
    hasData2 = false;
  }
  
  ws.cleanupClients();
}