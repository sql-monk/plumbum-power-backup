#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <time.h>

#include "config.h"
#include "dps5015.h"
#include "autopilot.h"

// Global objects
AsyncWebServer server(80);
DPS5015 dps(DPS_TX_PIN, DPS_RX_PIN, DPS_MODBUS_ADDRESS);
Adafruit_INA219 ina219;
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
Autopilot autopilot;

// Sensor data
bool dpsConnected = false;
bool ina219Available = false;
int numTempSensors = 0;

float dpsVin = 0.0;
float dpsCin = 0.0;
float dpsVout = 0.0;
float dpsCout = 0.0;
float dpsVset = 0.0;
float dpsCset = 0.0;
float inaVoltage = 0.0;
float acsCurrent = 0.0;
float temp1 = 25.0;
float temp2 = 25.0;

// GPIO states
bool pinD3State = false;
bool pinD4State = false;
bool pinD5State = false;

// Timing
unsigned long lastUpdate = 0;
unsigned long startTime = 0;

// Forward declarations
void setupWiFi();
void setupNTP();
void setupSensors();
void setupWebServer();
void updateSensors();
void updateAutopilot();
String getUptime();
String getNTPTime();
void handleRoot(AsyncWebServerRequest *request);
void handleData(AsyncWebServerRequest *request);
void handleCmd(AsyncWebServerRequest *request);

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ESP8266 Battery Charger Controller ===");
  
  // Step 1: Initialize GPIO
  Serial.println("Step 1: Initializing GPIO...");
  pinMode(GPIO_D3, OUTPUT);
  pinMode(GPIO_D4, OUTPUT);
  pinMode(GPIO_D5, OUTPUT);
  digitalWrite(GPIO_D3, LOW);
  digitalWrite(GPIO_D4, LOW);
  digitalWrite(GPIO_D5, LOW);
  
  // Step 2: Setup WiFi
  Serial.println("Step 2: Setting up WiFi...");
  setupWiFi();
  
  // Step 3: Setup NTP
  Serial.println("Step 3: Setting up NTP...");
  setupNTP();
  
  // Step 4: Initialize sensors
  Serial.println("Step 4: Initializing sensors...");
  setupSensors();
  
  // Step 5: Initialize autopilot
  Serial.println("Step 5: Initializing autopilot...");
  autopilot.begin();
  
  // Step 6: Setup web server
  Serial.println("Step 6: Setting up web server...");
  setupWebServer();
  
  startTime = millis();
  Serial.println("=== Setup complete ===");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentMillis;
    
    // Update all sensors
    updateSensors();
    
    // Update autopilot
    updateAutopilot();
  }
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(STATIC_IP, GATEWAY, SUBNET);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" Failed to connect!");
  }
}

void setupNTP() {
  configTime(NTP_TIMEZONE_OFFSET, 0, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
  Serial.println("Waiting for NTP time sync...");
  delay(2000);
}

void setupSensors() {
  // Initialize I2C
  Wire.begin(INA219_SDA, INA219_SCL);
  
  // Initialize DPS5015
  Serial.print("Initializing DPS5015... ");
  dpsConnected = dps.begin();
  if (dpsConnected) {
    Serial.println("OK");
    // Set initial current limit
    dps.setCurrent(MAX_CURRENT);
  } else {
    Serial.println("FAILED");
  }
  
  // Initialize INA219
  Serial.print("Initializing INA219... ");
  if (ina219.begin()) {
    ina219Available = true;
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }
  
  // Initialize DS18B20
  Serial.print("Initializing DS18B20... ");
  sensors.begin();
  numTempSensors = sensors.getDeviceCount();
  Serial.print(numTempSensors);
  Serial.println(" sensor(s) found");
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/cmd", HTTP_GET, handleCmd);
  
  server.begin();
  Serial.println("Web server started");
}

void updateSensors() {
  // Read DPS5015
  if (dpsConnected) {
    dpsVin = dps.readInputVoltage();
    dpsCin = dps.readInputCurrent();
    dpsVout = dps.readOutputVoltage();
    dpsCout = dps.readOutputCurrent();
    dpsVset = dps.readSetVoltage();
    dpsCset = dps.readSetCurrent();
  }
  
  // Read INA219
  if (ina219Available) {
    inaVoltage = ina219.getBusVoltage_V();
  }
  
  // Read ACS711
  int adcValue = analogRead(ACS711_PIN);
  float voltage = adcValue * 5.0 / 1023.0;
  acsCurrent = (voltage - ACS711_ZERO_OFFSET) / ACS711_SENSITIVITY;
  
  // Apply deadband
  if (abs(acsCurrent) < CURRENT_DEADBAND) {
    acsCurrent = 0.0;
  }
  
  // Read DS18B20
  sensors.requestTemperatures();
  if (numTempSensors > 0) {
    temp1 = sensors.getTempCByIndex(0);
    if (temp1 < TEMP_MIN || temp1 > TEMP_MAX) {
      temp1 = REFERENCE_TEMP;
    }
  }
  if (numTempSensors > 1) {
    temp2 = sensors.getTempCByIndex(1);
    if (temp2 < TEMP_MIN || temp2 > TEMP_MAX) {
      temp2 = REFERENCE_TEMP;
    }
  }
}

void updateAutopilot() {
  // Calculate average temperature
  float avgTemp = temp1;
  if (numTempSensors > 1) {
    avgTemp = (temp1 + temp2) / 2.0;
  }
  
  // Update autopilot state
  autopilot.update(inaVoltage, acsCurrent, avgTemp);
  
  // Apply autopilot settings to DPS
  if (dpsConnected) {
    ChargerState state = autopilot.getState();
    
    if (state == STATE_CHARGING) {
      dps.setVoltage(autopilot.getChargeVoltage());
      dps.setCurrent(autopilot.getChargeCurrent());
    } else if (state == STATE_STANDBY || state == STATE_DISCHARGING) {
      dps.setVoltage(autopilot.getStandbyVoltage());
      dps.setCurrent(autopilot.getStandbyCurrent());
    } else if (state == STATE_DISCONNECTED) {
      dps.setVoltage(autopilot.getStandbyVoltage());
      dps.setCurrent(autopilot.getStandbyCurrent());
    }
  }
}

String getUptime() {
  unsigned long uptime = (millis() - startTime) / 1000;
  unsigned long hours = uptime / 3600;
  unsigned long minutes = (uptime % 3600) / 60;
  unsigned long seconds = uptime % 60;
  
  String result = "";
  if (hours > 0) {
    result += String(hours) + "h ";
  }
  result += String(minutes) + "m ";
  result += String(seconds) + "s";
  
  return result;
}

String getNTPTime() {
  time_t now = time(nullptr);
  if (now < 1000000) {
    return "Waiting for NTP...";
  }
  
  struct tm* timeinfo = localtime(&now);
  char buffer[20];
  sprintf(buffer, "%02d.%02d.%04d %02d:%02d:%02d",
          timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900,
          timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
  
  return String(buffer);
}

void handleRoot(AsyncWebServerRequest *request) {
  Serial.println("GET /");
  
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  
  response->print("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
  response->print("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  response->print("<title>Battery Charger Controller</title>");
  response->print("<style>");
  response->print("* { margin: 0; padding: 0; box-sizing: border-box; }");
  response->print("body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;");
  response->print("background: #0d1117; color: #c9d1d9; padding: 20px; }");
  response->print(".container { max-width: 1200px; margin: 0 auto; }");
  response->print("h1 { color: #58a6ff; margin-bottom: 20px; text-align: center; }");
  response->print(".row { display: grid; grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));");
  response->print("gap: 20px; margin-bottom: 20px; }");
  response->print(".block { background: #161b22; border: 1px solid #30363d; border-radius: 6px;");
  response->print("padding: 20px; transition: box-shadow 0.3s ease; }");
  response->print(".block.glow-green { box-shadow: 0 0 20px rgba(46, 160, 67, 0.5); }");
  response->print(".block.glow-red { box-shadow: 0 0 20px rgba(248, 81, 73, 0.5); }");
  response->print(".block.glow-blue { box-shadow: 0 0 20px rgba(88, 166, 255, 0.5); }");
  response->print(".block.glow-orange { box-shadow: 0 0 20px rgba(219, 109, 40, 0.5); }");
  response->print(".block h2 { color: #58a6ff; font-size: 18px; margin-bottom: 15px; }");
  response->print(".metric { display: flex; justify-content: space-between; padding: 8px 0;");
  response->print("border-bottom: 1px solid #21262d; }");
  response->print(".metric:last-child { border-bottom: none; }");
  response->print(".metric label { color: #8b949e; }");
  response->print(".metric value { color: #c9d1d9; font-weight: bold; }");
  response->print(".state-badge { display: inline-block; padding: 4px 12px; border-radius: 12px;");
  response->print("font-size: 12px; font-weight: bold; }");
  response->print(".state-STANDBY { background: #238636; color: white; }");
  response->print(".state-CHARGING { background: #1f6feb; color: white; }");
  response->print(".state-DISCHARGING { background: #db6d28; color: white; }");
  response->print(".state-DISCONNECTED { background: #6e7681; color: white; }");
  response->print(".state-ERROR { background: #da3633; color: white; }");
  response->print("button { background: #238636; color: white; border: none; padding: 8px 16px;");
  response->print("border-radius: 6px; cursor: pointer; font-size: 14px; transition: background 0.2s; }");
  response->print("button:hover { background: #2ea043; }");
  response->print("button.off { background: #da3633; }");
  response->print("button.off:hover { background: #f85149; }");
  response->print("input[type='number'] { background: #0d1117; color: #c9d1d9; border: 1px solid #30363d;");
  response->print("padding: 6px 12px; border-radius: 6px; width: 100px; }");
  response->print(".control-group { display: flex; align-items: center; gap: 10px; margin-bottom: 10px; }");
  response->print(".control-group label { flex: 1; color: #8b949e; }");
  response->print(".control-cols { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }");
  response->print("@media (max-width: 768px) { .row, .control-cols { grid-template-columns: 1fr; } }");
  response->print("</style></head><body>");
  
  response->print("<div class='container'>");
  response->print("<h1>⚡ Battery Charger Controller</h1>");
  
  response->print("<div class='row'>");
  response->print("<div class='block' id='dps-block'>");
  response->print("<h2>🔌 DPS Power Supply</h2>");
  response->print("<div class='metric'><label>Status:</label><value id='dps-status'>-</value></div>");
  response->print("<div class='metric'><label>Input:</label><value id='dps-vin'>- V / - A</value></div>");
  response->print("<div class='metric'><label>Output:</label><value id='dps-vout'>- V / - A</value></div>");
  response->print("<div class='metric'><label>Set:</label><value id='dps-set'>- V / - A</value></div>");
  response->print("<button id='dps-btn' onclick='toggleDPS()'>Turn ON</button>");
  response->print("</div>");
  
  response->print("<div class='block' id='battery-block'>");
  response->print("<h2>🔋 Battery Status</h2>");
  response->print("<div class='metric'><label>Voltage:</label><value id='bat-v'>- V</value></div>");
  response->print("<div class='metric'><label>Current:</label><value id='bat-c'>- A</value></div>");
  response->print("<div class='metric'><label>Temp 1:</label><value id='temp1'>- °C</value></div>");
  response->print("<div class='metric'><label>Temp 2:</label><value id='temp2'>- °C</value></div>");
  response->print("<div class='metric'><label>Avg Temp:</label><value id='temp-avg'>- °C</value></div>");
  response->print("<div class='metric'><label>Temp Comp:</label><value id='temp-comp'>- V</value></div>");
  response->print("<div class='metric'><label>State:</label><value><span class='state-badge' id='state'>-</span></value></div>");
  response->print("</div>");
  response->print("</div>");
  
  response->print("<div class='block'>");
  response->print("<h2>⚙️ Control Panel</h2>");
  response->print("<div class='control-cols'>");
  
  response->print("<div>");
  response->print("<h3 style='color: #58a6ff; font-size: 16px; margin-bottom: 10px;'>Charge Settings</h3>");
  response->print("<div class='control-group'><label>Bulk Voltage:</label>");
  response->print("<input type='number' id='charge-v' step='0.1' min='20' max='29'>");
  response->print("<button onclick='setChargeV()'>SET</button></div>");
  response->print("<div class='control-group'><label>Max Current:</label>");
  response->print("<input type='number' id='charge-c' step='0.1' min='0' max='10'>");
  response->print("<button onclick='setChargeC()'>SET</button></div>");
  response->print("<div class='control-group'><label>Standby Voltage:</label>");
  response->print("<input type='number' id='standby-v' step='0.1' min='20' max='29'>");
  response->print("<button onclick='setStandbyV()'>SET</button></div>");
  response->print("<div class='control-group'><label>Standby Current:</label>");
  response->print("<input type='number' id='standby-c' step='0.1' min='0' max='10'>");
  response->print("<button onclick='setStandbyC()'>SET</button></div>");
  response->print("</div>");
  
  response->print("<div>");
  response->print("<h3 style='color: #58a6ff; font-size: 16px; margin-bottom: 10px;'>GPIO Control</h3>");
  response->print("<div class='control-group'><label>D3 (Relay):</label>");
  response->print("<button id='btn-d3' onclick='togglePin(\"d3\")'>OFF</button></div>");
  response->print("<div class='control-group'><label>D4:</label>");
  response->print("<button id='btn-d4' onclick='togglePin(\"d4\")'>OFF</button></div>");
  response->print("<div class='control-group'><label>D5:</label>");
  response->print("<button id='btn-d5' onclick='togglePin(\"d5\")'>OFF</button></div>");
  response->print("<div style='margin-top: 20px;'>");
  response->print("<div class='metric'><label>Uptime:</label><value id='uptime'>-</value></div>");
  response->print("<div class='metric'><label>Date/Time:</label><value id='datetime'>-</value></div>");
  response->print("</div>");
  response->print("</div>");
  
  response->print("</div>");
  response->print("</div>");
  response->print("</div>");
  
  response->print("<script>");
  response->print("let dpsOutput = false;");
  response->print("function updateData() {");
  response->print("  fetch('/data').then(r => r.text()).then(data => {");
  response->print("    const parts = data.split(';');");
  response->print("    const dpsConn = parts[0] === '1';");
  response->print("    document.getElementById('dps-status').textContent = dpsConn ? 'Connected' : 'Disconnected';");
  response->print("    document.getElementById('dps-vin').textContent = parts[1] + ' V / ' + parts[2] + ' A';");
  response->print("    document.getElementById('dps-vout').textContent = parts[3] + ' V / ' + parts[4] + ' A';");
  response->print("    document.getElementById('dps-set').textContent = parts[5] + ' V / ' + parts[6] + ' A';");
  response->print("    document.getElementById('bat-v').textContent = parts[7] + ' V';");
  response->print("    document.getElementById('bat-c').textContent = parts[8] + ' A';");
  response->print("    document.getElementById('temp1').textContent = parts[9] + ' °C';");
  response->print("    document.getElementById('temp2').textContent = parts[10] + ' °C';");
  response->print("    const t1 = parseFloat(parts[9]); const t2 = parseFloat(parts[10]);");
  response->print("    const avgTemp = isNaN(t1) ? 25 : (isNaN(t2) ? t1 : ((t1+t2)/2));");
  response->print("    document.getElementById('temp-avg').textContent = avgTemp.toFixed(1) + ' °C';");
  response->print("    const tempComp = (avgTemp - 25) * -0.036;");
  response->print("    document.getElementById('temp-comp').textContent = tempComp.toFixed(2) + ' V';");
  response->print("    const state = parts[11];");
  response->print("    const stateBadge = document.getElementById('state');");
  response->print("    stateBadge.textContent = state;");
  response->print("    stateBadge.className = 'state-badge state-' + state;");
  response->print("    document.getElementById('charge-v').value = parts[12];");
  response->print("    document.getElementById('charge-c').value = parts[13];");
  response->print("    document.getElementById('standby-v').value = parts[14];");
  response->print("    document.getElementById('standby-c').value = parts[15];");
  response->print("    updatePinButton('d3', parts[16] === '1');");
  response->print("    updatePinButton('d4', parts[17] === '1');");
  response->print("    updatePinButton('d5', parts[18] === '1');");
  response->print("    document.getElementById('uptime').textContent = parts[19];");
  response->print("    document.getElementById('datetime').textContent = parts[20];");
  response->print("    const dpsBlock = document.getElementById('dps-block');");
  response->print("    dpsBlock.className = 'block ' + (dpsConn ? 'glow-green' : 'glow-red');");
  response->print("    const batBlock = document.getElementById('battery-block');");
  response->print("    let glow = 'glow-green';");
  response->print("    if (state === 'CHARGING') glow = 'glow-blue';");
  response->print("    else if (state === 'DISCHARGING') glow = 'glow-orange';");
  response->print("    else if (state === 'ERROR' || state === 'DISCONNECTED') glow = 'glow-red';");
  response->print("    batBlock.className = 'block ' + glow;");
  response->print("  }).catch(e => console.error('Error:', e));");
  response->print("}");
  response->print("function toggleDPS() {");
  response->print("  dpsOutput = !dpsOutput;");
  response->print("  const action = dpsOutput ? 'dps_output_on' : 'dps_output_off';");
  response->print("  fetch('/cmd?action=' + action).then(() => {");
  response->print("    const btn = document.getElementById('dps-btn');");
  response->print("    btn.textContent = dpsOutput ? 'Turn OFF' : 'Turn ON';");
  response->print("    btn.className = dpsOutput ? 'off' : '';");
  response->print("  });");
  response->print("}");
  response->print("function setChargeV() { const v = document.getElementById('charge-v').value;");
  response->print("  fetch('/cmd?action=set_charge_voltage&value=' + v); }");
  response->print("function setChargeC() { const c = document.getElementById('charge-c').value;");
  response->print("  fetch('/cmd?action=set_charge_current&value=' + c); }");
  response->print("function setStandbyV() { const v = document.getElementById('standby-v').value;");
  response->print("  fetch('/cmd?action=set_standby_voltage&value=' + v); }");
  response->print("function setStandbyC() { const c = document.getElementById('standby-c').value;");
  response->print("  fetch('/cmd?action=set_standby_current&value=' + c); }");
  response->print("function togglePin(pin) { fetch('/cmd?action=toggle_pin&pin=' + pin); }");
  response->print("function updatePinButton(pin, state) {");
  response->print("  const btn = document.getElementById('btn-' + pin);");
  response->print("  btn.textContent = state ? 'ON' : 'OFF';");
  response->print("  btn.className = state ? 'off' : '';");
  response->print("}");
  response->print("setInterval(updateData, 1000);");
  response->print("updateData();");
  response->print("</script>");
  
  response->print("</body></html>");
  
  request->send(response);
}

void handleData(AsyncWebServerRequest *request) {
  // Build data string: all values separated by semicolons
  String data = "";
  data += String(dpsConnected ? 1 : 0) + ";";
  data += String(dpsVin, 2) + ";";
  data += String(dpsCin, 2) + ";";
  data += String(dpsVout, 2) + ";";
  data += String(dpsCout, 2) + ";";
  data += String(dpsVset, 2) + ";";
  data += String(dpsCset, 2) + ";";
  data += String(inaVoltage, 2) + ";";
  data += String(acsCurrent, 2) + ";";
  data += String(temp1, 1) + ";";
  data += String(temp2, 1) + ";";
  data += autopilot.getStateString() + String(";");
  data += String(autopilot.getChargeVoltage(), 2) + ";";
  data += String(autopilot.getChargeCurrent(), 2) + ";";
  data += String(autopilot.getStandbyVoltage(), 2) + ";";
  data += String(autopilot.getStandbyCurrent(), 2) + ";";
  data += String(pinD3State ? 1 : 0) + ";";
  data += String(pinD4State ? 1 : 0) + ";";
  data += String(pinD5State ? 1 : 0) + ";";
  data += getUptime() + ";";
  data += getNTPTime();
  
  request->send(200, "text/plain", data);
}

void handleCmd(AsyncWebServerRequest *request) {
  if (!request->hasParam("action")) {
    request->send(400, "text/plain", "Missing action parameter");
    return;
  }
  
  String action = request->getParam("action")->value();
  Serial.print("GET /cmd?action=");
  Serial.println(action);
  
  if (action == "dps_output_on") {
    if (dpsConnected) {
      dps.setOutput(true);
    }
    request->send(200, "text/plain", "OK");
  }
  else if (action == "dps_output_off") {
    if (dpsConnected) {
      dps.setOutput(false);
    }
    request->send(200, "text/plain", "OK");
  }
  else if (action == "set_charge_voltage") {
    if (request->hasParam("value")) {
      float value = request->getParam("value")->value().toFloat();
      autopilot.setChargeVoltage(value);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing value");
    }
  }
  else if (action == "set_charge_current") {
    if (request->hasParam("value")) {
      float value = request->getParam("value")->value().toFloat();
      autopilot.setChargeCurrent(value);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing value");
    }
  }
  else if (action == "set_standby_voltage") {
    if (request->hasParam("value")) {
      float value = request->getParam("value")->value().toFloat();
      autopilot.setStandbyVoltage(value);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing value");
    }
  }
  else if (action == "set_standby_current") {
    if (request->hasParam("value")) {
      float value = request->getParam("value")->value().toFloat();
      autopilot.setStandbyCurrent(value);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing value");
    }
  }
  else if (action == "toggle_pin") {
    if (request->hasParam("pin")) {
      String pin = request->getParam("pin")->value();
      if (pin == "d3") {
        pinD3State = !pinD3State;
        digitalWrite(GPIO_D3, pinD3State ? HIGH : LOW);
      } else if (pin == "d4") {
        pinD4State = !pinD4State;
        digitalWrite(GPIO_D4, pinD4State ? HIGH : LOW);
      } else if (pin == "d5") {
        pinD5State = !pinD5State;
        digitalWrite(GPIO_D5, pinD5State ? HIGH : LOW);
      }
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing pin");
    }
  }
  else {
    request->send(400, "text/plain", "Unknown action");
  }
}
