/*
  ESP32 Weather Station - Basic Sensor Test

  Reads:
  - BME280 temperature / humidity / pressure
  - BH1750 light level
  - INA219 voltage / current / power
  - RS485 wind speed sensor
  - RS485 wind direction sensor

  Board:
  ESP32D Dev Board

  I2C:
    SDA = GPIO21
    SCL = GPIO22

  Wind Speed MAX485:
    RX  = GPIO16
    TX  = GPIO17
    DE/RE = GPIO4

  Wind Direction MAX485:
    RX  = GPIO33
    TX  = GPIO32
    DE/RE = GPIO27

  Serial Monitor:
    115200 baud
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <Adafruit_INA219.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ----------------------------------------------------
// WIFI SETTINGS
// ----------------------------------------------------
const char* WIFI_SSID = "REMOVED_SSID";
const char* WIFI_PASSWORD = "REMOVED_WIFI_PASSWORD";

WebServer server(80);

// ----------------------------------------------------
// LATEST WEATHER DATA FOR WEB DASHBOARD
// ----------------------------------------------------
float latestTemperature = 0;
float latestHumidity = 0;
float latestPressure = 0;
float latestLux = 0;

float latestLoadVoltage = 0;
float latestLoadCurrentMA = 0;
float latestLoadPowerMW = 0;

float latestSolarVoltage = 0;
float latestSolarCurrentMA = 0;
float latestSolarPowerMW = 0;

float latestNetCurrentMA = 0;
String latestBatteryState = "unknown";

float latestWindMS = 0;
float latestWindKPH = 0;
float latestWindMPH = 0;

float latestWindAvgMS = 0;
float latestWindAvgKPH = 0;
float latestWindAvgMPH = 0;

float latestWindGustMS = 0;
float latestWindGustKPH = 0;
float latestWindGustMPH = 0;

String latestWindDirName = "ERR";
float latestWindDirDegrees = -1;
uint16_t latestWindDirRaw = 0;

bool latestBmeOK = false;
bool latestBh1750OK = false;
bool latestLoadOK = false;
bool latestSolarOK = false;
bool latestWindSpeedOK = false;
bool latestWindDirOK = false;

unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_UPDATE_INTERVAL = 2000;

float latestBatteryVoltage = 0;
int latestBatteryPercent = 0;
String latestIPAddress = "unknown";

//----------------------------------------------------
//Wifi strength globals
//----------------------------------------------------
int latestWiFiRSSI = 0;
int latestWiFiPercent = 0;
String latestWiFiQuality = "unknown";

//----Power switch-----
bool heltecPowerOn = false;

// ----------------------------------------------------
// BATTERY ADC
// ----------------------------------------------------
#define BATTERY_ADC_PIN 34

// Your divider is 100k / 100k, so ADC voltage is half battery voltage.
#define BATTERY_DIVIDER_RATIO 2.0

// ESP32 ADC reference is not perfect, so this may need calibration later.
#define ADC_REF_VOLTAGE 3.3
#define ADC_MAX_READING 4095.0

// 1S Li-ion rough voltage range
#define BATTERY_FULL_VOLTAGE 4.1
#define BATTERY_EMPTY_VOLTAGE 3.20

// ----------------------------------------------------
// I2C PINS
// ----------------------------------------------------
#define I2C_SDA 21
#define I2C_SCL 22

// ----------------------------------------------------
// OPTIONAL HELTEC POWER CONTROL
// ----------------------------------------------------
#define HELTEC_POWER_PIN 25

// Set this depending on your power control circuit.
// For now we leave it LOW in setup.
#define HELTEC_POWER_DEFAULT LOW

// ----------------------------------------------------
// RS485 WIND SPEED
// ----------------------------------------------------
#define WIND_SPEED_RX     16
#define WIND_SPEED_TX     17
#define WIND_SPEED_DE_RE  4

// ----------------------------------------------------
// RS485 WIND DIRECTION
// ----------------------------------------------------
#define WIND_DIR_RX       33
#define WIND_DIR_TX       32
#define WIND_DIR_DE_RE    27

// ----------------------------------------------------
// MODBUS SETTINGS
// ----------------------------------------------------
#define MODBUS_ADDRESS    1
#define MODBUS_BAUD       9600
#define WIND_REGISTER     0x0000

// ----------------------------------------------------
// WIND AVERAGING / GUST TRACKING
// ----------------------------------------------------
#define WIND_SAMPLE_COUNT 30

float windSamples[WIND_SAMPLE_COUNT];
int windSampleIndex = 0;
int windSampleFilled = 0;

float windGustMS = 0;
float windGustKPH = 0;
float windGustMPH = 0;

unsigned long lastGustReset = 0;
const unsigned long GUST_RESET_INTERVAL = 60UL * 60UL * 1000UL; // 1 hour

// ----------------------------------------------------
// SENSOR OBJECTS
// ----------------------------------------------------
Adafruit_BME280 bme;
BH1750 lightMeter;
Adafruit_INA219 inaLoad(0x40);
Adafruit_INA219 inaSolar(0x44);

HardwareSerial WindSpeedSerial(1);
HardwareSerial WindDirSerial(2);

ModbusMaster windSpeed;
ModbusMaster windDirection;

// ----------------------------------------------------
// SENSOR STATUS FLAGS
// ----------------------------------------------------
bool bmeOK = false;
bool bh1750OK = false;
bool inaLoadOK = false;
bool inaSolarOK = false;

// ----------------------------------------------------
// MAX485 direction control - wind speed
// ----------------------------------------------------
void preTransmissionSpeed() {
  digitalWrite(WIND_SPEED_DE_RE, HIGH);
  delayMicroseconds(300);
}

void postTransmissionSpeed() {
  delayMicroseconds(300);
  digitalWrite(WIND_SPEED_DE_RE, LOW);
}

// ----------------------------------------------------
// MAX485 direction control - wind direction
// ----------------------------------------------------
void preTransmissionDirection() {
  digitalWrite(WIND_DIR_DE_RE, HIGH);
  delayMicroseconds(300);
}

void postTransmissionDirection() {
  delayMicroseconds(300);
  digitalWrite(WIND_DIR_DE_RE, LOW);
}

// ----------------------------------------------------
// 16-point compass helper
// ----------------------------------------------------
String compass16Name(uint16_t raw) {
  switch (raw) {
    case 0:  return "N";
    case 1:  return "NNE";
    case 2:  return "NE";
    case 3:  return "ENE";
    case 4:  return "E";
    case 5:  return "ESE";
    case 6:  return "SE";
    case 7:  return "SSE";
    case 8:  return "S";
    case 9:  return "SSW";
    case 10: return "SW";
    case 11: return "WSW";
    case 12: return "W";
    case 13: return "WNW";
    case 14: return "NW";
    case 15: return "NNW";
    default: return "ERR";
  }
}

float compass16Degrees(uint16_t raw) {
  if (raw > 15) return -1;
  return raw * 22.5;
}

// ----------------------------------------------------
// I2C scanner
// ----------------------------------------------------
void scanI2C() {
  Serial.println();
  Serial.println("Scanning I2C bus...");

  byte count = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);

      if (address == 0x76 || address == 0x77) Serial.print("  likely BME280/BMP280");
      if (address == 0x23 || address == 0x5C) Serial.print("  likely BH1750");
      if (address == 0x40) Serial.print("  likely INA219");

      Serial.println();
      count++;
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" I2C device(s).");
  }
}

// ----------------------------------------------------
// Start BME280
// ----------------------------------------------------
void setupBME280() {
  Serial.println();
  Serial.println("Starting BME280...");

  if (bme.begin(0x76)) {
    bmeOK = true;
    Serial.println("BME280 found at 0x76.");
    return;
  }

  if (bme.begin(0x77)) {
    bmeOK = true;
    Serial.println("BME280 found at 0x77.");
    return;
  }

  bmeOK = false;
  Serial.println("BME280 not found at 0x76 or 0x77.");
}

// ----------------------------------------------------
// Start BH1750
// ----------------------------------------------------
void setupBH1750() {
  Serial.println();
  Serial.println("Starting BH1750...");

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    bh1750OK = true;
    Serial.println("BH1750 started.");
  } else {
    bh1750OK = false;
    Serial.println("BH1750 not found / failed to start.");
  }
}

// ----------------------------------------------------
// Start INA219
// ----------------------------------------------------
void setupINA219() {
  Serial.println();
  Serial.println("Starting INA219 sensors...");

  if (inaLoad.begin()) {
    inaLoadOK = true;
    Serial.println("INA219 Load sensor found at 0x40.");
  } else {
    inaLoadOK = false;
    Serial.println("INA219 Load sensor not found at 0x40.");
  }

  if (inaSolar.begin()) {
    inaSolarOK = true;
    Serial.println("INA219 Solar charge sensor found at 0x44.");
  } else {
    inaSolarOK = false;
    Serial.println("INA219 Solar charge sensor not found at 0x44.");
  }
}

// ----------------------------------------------------
// Start RS485 wind sensors
// ----------------------------------------------------
void setupWindSensors() {
  Serial.println();
  Serial.println("Starting RS485 wind sensors...");

  pinMode(WIND_SPEED_DE_RE, OUTPUT);
  pinMode(WIND_DIR_DE_RE, OUTPUT);

  digitalWrite(WIND_SPEED_DE_RE, LOW);
  digitalWrite(WIND_DIR_DE_RE, LOW);

  WindSpeedSerial.begin(MODBUS_BAUD, SERIAL_8N1, WIND_SPEED_RX, WIND_SPEED_TX);
  WindDirSerial.begin(MODBUS_BAUD, SERIAL_8N1, WIND_DIR_RX, WIND_DIR_TX);

  windSpeed.begin(MODBUS_ADDRESS, WindSpeedSerial);
  windSpeed.preTransmission(preTransmissionSpeed);
  windSpeed.postTransmission(postTransmissionSpeed);

  windDirection.begin(MODBUS_ADDRESS, WindDirSerial);
  windDirection.preTransmission(preTransmissionDirection);
  windDirection.postTransmission(postTransmissionDirection);

  Serial.println("Wind speed RS485 ready on RX16 / TX17 / DE4.");
  Serial.println("Wind direction RS485 ready on RX33 / TX32 / DE27.");
}

// ----------------------------------------------------
// Read wind speed
// ----------------------------------------------------
bool readWindSpeed(float &ms, float &kph, float &mph, uint16_t &raw) {
  uint8_t result = windSpeed.readHoldingRegisters(WIND_REGISTER, 1);

  if (result == windSpeed.ku8MBSuccess) {
    raw = windSpeed.getResponseBuffer(0);

    // Confirmed from your test:
    // raw / 10 = m/s
    ms = raw / 10.0;
    kph = ms * 3.6;
    mph = ms * 2.23694;

    return true;
  }

  return false;
}

// ----------------------------------------------------
// Read wind direction
// ----------------------------------------------------
bool readWindDirection(uint16_t &raw, String &name, float &degrees) {
  uint8_t result = windDirection.readHoldingRegisters(WIND_REGISTER, 1);

  if (result == windDirection.ku8MBSuccess) {
    raw = windDirection.getResponseBuffer(0);

    name = compass16Name(raw);
    degrees = compass16Degrees(raw);

    return true;
  }

  return false;
}

// ----------------------------------------------------
// Wind Actions Helper
// ----------------------------------------------------
void addWindSample(float windMS) {
  windSamples[windSampleIndex] = windMS;

  windSampleIndex++;

  if (windSampleIndex >= WIND_SAMPLE_COUNT) {
    windSampleIndex = 0;
  }

  if (windSampleFilled < WIND_SAMPLE_COUNT) {
    windSampleFilled++;
  }

  if (windMS > windGustMS) {
    windGustMS = windMS;
    windGustKPH = windMS * 3.6;
    windGustMPH = windMS * 2.23694;
  }
}

float getAverageWindMS() {
  if (windSampleFilled == 0) {
    return 0;
  }

  float total = 0;

  for (int i = 0; i < windSampleFilled; i++) {
    total += windSamples[i];
  }

  return total / windSampleFilled;
}

void resetGustIfNeeded() {
  if (millis() - lastGustReset >= GUST_RESET_INTERVAL) {
    lastGustReset = millis();

    windGustMS = 0;
    windGustKPH = 0;
    windGustMPH = 0;

    Serial.println("Wind gust reset.");
  }
}

//------------------------------------
//Embedded WebPage 
//------------------------------------
const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HandiWorx Weather Station</title>
  <style>
    body {
      margin: 0;
      font-family: Arial, Helvetica, sans-serif;
      background: #10131a;
      color: #f4f7fb;
    }

    header {
      padding: 20px;
      text-align: center;
      background: linear-gradient(135deg, #131722, #20283a);
      border-bottom: 1px solid #2f3a52;
    }

    h1 {
      margin: 0;
      font-size: 28px;
      letter-spacing: 1px;
    }

    .subtitle {
      margin-top: 6px;
      color: #9fb1c9;
      font-size: 14px;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(210px, 1fr));
      gap: 14px;
      padding: 14px;
      max-width: 1200px;
      margin: auto;
    }

    .card {
      background: #171d2a;
      border: 1px solid #2d374c;
      border-radius: 14px;
      padding: 16px;
      box-shadow: 0 8px 20px rgba(0,0,0,0.25);
    }

    .card h2 {
      margin: 0 0 12px 0;
      font-size: 15px;
      color: #8be9fd;
      text-transform: uppercase;
      letter-spacing: 1px;
    }

    .value {
      font-size: 30px;
      font-weight: bold;
      margin: 4px 0;
    }

    .unit {
      font-size: 15px;
      color: #aab7ca;
      margin-left: 4px;
    }
    .btn {
      border: none;
      border-radius: 10px;
      padding: 12px 16px;
      margin: 8px 6px 0 0;
      font-size: 15px;
      font-weight: bold;
      cursor: pointer;
      color: #10131a;
    }

    .btn.on {
      background: #6dff9b;
    }

    .btn.off {
      background: #ff7b7b;
    }
    .small {
      font-size: 14px;
      color: #b8c4d6;
      line-height: 1.5;
    }

    .status {
      margin-top: 10px;
      padding: 8px;
      border-radius: 8px;
      background: #20283a;
      color: #d5deec;
      font-size: 14px;
    }

    .charging {
      color: #6dff9b;
    }

    .discharging {
      color: #ff7b7b;
    }

    .balanced {
      color: #ffd36d;
    }

    footer {
      text-align: center;
      color: #6f7f98;
      padding: 18px;
      font-size: 13px;
    }
  </style>
</head>
<body>
  <header>
    <h1>HandiWorx Weather Station</h1>
    <div class="subtitle">
    ESP32 Solar Weather Node<br>
    IP: <span id="ip_address">--</span>
</div>
  </header>

  <main class="grid">
    <div class="card">
      <h2>Temperature</h2>
      <div class="value"><span id="temperature">--</span><span class="unit">°C</span></div>
      <div class="small">Humidity: <span id="humidity">--</span> %</div>
      <div class="small">Pressure: <span id="pressure">--</span> hPa</div>
    </div>

    <div class="card">
      <h2>Light</h2>
      <div class="value"><span id="lux">--</span><span class="unit">lux</span></div>
      <div class="small">BH1750 light sensor</div>
    </div>
    <div class="card">
      <h2>Wi-Fi Signal</h2>
      <div class="value"><span id="wifi_percent">--</span><span class="unit">%</span></div>
      <div class="small">RSSI: <span id="wifi_rssi">--</span> dBm</div>
      <div class="status">Quality: <span id="wifi_quality">--</span></div>
    </div>

    <div class="card">
  <h2>MeshCore Heltec</h2>
  <div class="value"><span id="heltec_power">--</span></div>
  <div class="small">Remote power control on GPIO25</div>
  <button class="btn on" onclick="setHeltecPower('on')">Power ON</button>
  <button class="btn off" onclick="setHeltecPower('off')">Power OFF</button>
  </div>
    <div class="card">
      <h2>Wind Speed</h2>
      <div class="value"><span id="wind_mph">--</span><span class="unit">mph</span></div>
      <div class="small"><span id="wind_ms">--</span> m/s</div>
      <div class="small"><span id="wind_kph">--</span> km/h</div>
    </div>

    <div class="card">
      <h2>Wind Direction</h2>
      <div class="value"><span id="wind_dir">--</span></div>
      <div class="small"><span id="wind_deg">--</span> degrees</div>
    </div>

    <div class="card">
      <h2>Wind Average</h2>
      <div class="value"><span id="wind_avg_mph">--</span><span class="unit">mph</span></div>
      <div class="small"><span id="wind_avg_ms">--</span> m/s</div>
    </div>

    <div class="card">
      <h2>Wind Gust</h2>
      <div class="value"><span id="wind_gust_mph">--</span><span class="unit">mph</span></div>
      <div class="small"><span id="wind_gust_ms">--</span> m/s</div>
    </div>

    <div class="card">
      <h2>Station Load</h2>
      <div class="small">Voltage: <span id="load_voltage">--</span> V</div>
      <div class="small">Current: <span id="load_current">--</span> mA</div>
      <div class="small">Power: <span id="load_power">--</span> mW</div>
    </div>

    <div class="card">
      <h2>Solar Charge</h2>
      <div class="small">Voltage: <span id="solar_voltage">--</span> V</div>
      <div class="small">Current: <span id="solar_current">--</span> mA</div>
      <div class="small">Power: <span id="solar_power">--</span> mW</div>
      <div class="status">Battery: <span id="battery_state">--</span></div>
      <div class="small">Net current: <span id="net_current">--</span> mA</div>
    </div>
    <div class="card">
      <h2>Battery</h2>
      <div class="value"><span id="battery_percent">--</span><span class="unit">%</span></div>
      <div class="small">Voltage: <span id="battery_voltage">--</span> V</div>
      <div class="status">State: <span id="battery_state_2">--</span></div>
    </div>

  </main>

  <footer>
    Last update: <span id="last_update">--</span>
  </footer>

<script>
async function updateData() {
  try {
    const res = await fetch('/data');
    const d = await res.json();

    document.getElementById('temperature').textContent = d.temperature.toFixed(1);
    document.getElementById('humidity').textContent = d.humidity.toFixed(1);
    document.getElementById('pressure').textContent = d.pressure.toFixed(1);
    document.getElementById('lux').textContent = d.lux.toFixed(1);

    document.getElementById('wind_ms').textContent = d.wind_ms.toFixed(1);
    document.getElementById('wind_kph').textContent = d.wind_kph.toFixed(1);
    document.getElementById('wind_mph').textContent = d.wind_mph.toFixed(1);

    document.getElementById('wind_dir').textContent = d.wind_dir;
    document.getElementById('wind_deg').textContent = d.wind_deg.toFixed(1);

    document.getElementById('wind_avg_ms').textContent = d.wind_avg_ms.toFixed(1);
    document.getElementById('wind_avg_mph').textContent = d.wind_avg_mph.toFixed(1);

    document.getElementById('wind_gust_ms').textContent = d.wind_gust_ms.toFixed(1);
    document.getElementById('wind_gust_mph').textContent = d.wind_gust_mph.toFixed(1);

    document.getElementById('load_voltage').textContent = d.load_voltage.toFixed(3);
    document.getElementById('load_current').textContent = d.load_current.toFixed(2);
    document.getElementById('load_power').textContent = d.load_power.toFixed(2);

    document.getElementById('solar_voltage').textContent = d.solar_voltage.toFixed(3);
    document.getElementById('solar_current').textContent = d.solar_current.toFixed(2);
    document.getElementById('solar_power').textContent = d.solar_power.toFixed(2);

    document.getElementById('ip_address').textContent = d.ip_address;

    document.getElementById('wifi_percent').textContent = d.wifi_percent;
    document.getElementById('wifi_rssi').textContent = d.wifi_rssi;
    document.getElementById('wifi_quality').textContent = d.wifi_quality;

    document.getElementById('battery_voltage').textContent = d.battery_voltage.toFixed(3);
    document.getElementById('battery_percent').textContent = d.battery_percent;
    document.getElementById('battery_state_2').textContent = d.battery_state;

    const battery = document.getElementById('battery_state');
    battery.textContent = d.battery_state;
    battery.className = d.battery_state;

    document.getElementById('heltec_power').textContent =
      d.heltec_power ? d.heltec_power.toUpperCase() : 'UNKNOWN';

    document.getElementById('net_current').textContent = d.net_current.toFixed(2);

    document.getElementById('last_update').textContent = new Date().toLocaleTimeString();
  } catch (e) {
    document.getElementById('last_update').textContent = 'connection error';
  }
}
async function setHeltecPower(state) {
  try {
    await fetch('/heltec/' + state);
    updateData();
  } catch (e) {
    alert('Failed to change Heltec power');
  }
}
updateData();
setInterval(updateData, 2000);
</script>
</body>
</html>
)rawliteral";
//---------------------
//--END WEBPAGE CONTENT
//---------------------


//---------------------------------------
//Web Server Functions
//---------------------------------------
void handleRoot() {
  server.send_P(200, "text/html", MAIN_PAGE);
}

void handleData() {
  String json = "{";

  json += "\"temperature\":" + String(latestTemperature, 2) + ",";
  json += "\"humidity\":" + String(latestHumidity, 2) + ",";
  json += "\"pressure\":" + String(latestPressure, 2) + ",";
  json += "\"lux\":" + String(latestLux, 2) + ",";

  json += "\"wind_ms\":" + String(latestWindMS, 2) + ",";
  json += "\"wind_kph\":" + String(latestWindKPH, 2) + ",";
  json += "\"wind_mph\":" + String(latestWindMPH, 2) + ",";

  json += "\"wind_avg_ms\":" + String(latestWindAvgMS, 2) + ",";
  json += "\"wind_avg_kph\":" + String(latestWindAvgKPH, 2) + ",";
  json += "\"wind_avg_mph\":" + String(latestWindAvgMPH, 2) + ",";

  json += "\"wind_gust_ms\":" + String(latestWindGustMS, 2) + ",";
  json += "\"wind_gust_kph\":" + String(latestWindGustKPH, 2) + ",";
  json += "\"wind_gust_mph\":" + String(latestWindGustMPH, 2) + ",";

  json += "\"wind_dir\":\"" + latestWindDirName + "\",";
  json += "\"wind_deg\":" + String(latestWindDirDegrees, 2) + ",";

  json += "\"load_voltage\":" + String(latestLoadVoltage, 3) + ",";
  json += "\"load_current\":" + String(latestLoadCurrentMA, 2) + ",";
  json += "\"load_power\":" + String(latestLoadPowerMW, 2) + ",";

  json += "\"solar_voltage\":" + String(latestSolarVoltage, 3) + ",";
  json += "\"solar_current\":" + String(latestSolarCurrentMA, 2) + ",";
  json += "\"solar_power\":" + String(latestSolarPowerMW, 2) + ",";

  json += "\"net_current\":" + String(latestNetCurrentMA, 2) + ",";
  json += "\"battery_state\":\"" + latestBatteryState + "\",";
  json += "\"battery_voltage\":" + String(latestBatteryVoltage, 3) + ",";
  json += "\"battery_percent\":" + String(latestBatteryPercent) + ",";
  json += "\"ip_address\":\"" + latestIPAddress + "\",";

  json += "\"wifi_rssi\":" + String(latestWiFiRSSI) + ",";
  json += "\"wifi_percent\":" + String(latestWiFiPercent) + ",";
  json += "\"wifi_quality\":\"" + latestWiFiQuality + "\",";

  json += "\"heltec_power\":\"" + String(heltecPowerOn ? "on" : "off") + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void setupWiFiAndServer() {
  Serial.println();
  Serial.println("Starting WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
latestIPAddress = WiFi.localIP().toString();

    Serial.print("IP address: ");
    Serial.println(latestIPAddress);

    if (MDNS.begin("weatherstation")) {
      Serial.println("mDNS started: http://weatherstation.local/");
    }
  } else {
    Serial.println("WiFi failed. Starting fallback access point.");

    WiFi.mode(WIFI_AP);
    WiFi.softAP("WeatherStation", "weather123");

    latestIPAddress = WiFi.softAPIP().toString();

    Serial.print("Access Point IP: ");
    Serial.println(latestIPAddress);

    Serial.println("Connect to WiFi: WeatherStation");
    Serial.println("Password: weather123");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);

  server.on("/heltec/on", handleHeltecOn);
  server.on("/heltec/off", handleHeltecOff);

  server.begin();
  Serial.println("Web server started.");
}

void updateWeatherData() {

  if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
    latestWiFiRSSI = WiFi.RSSI();
    latestWiFiPercent = wifiPercentFromRSSI(latestWiFiRSSI);
    latestWiFiQuality = wifiQualityFromRSSI(latestWiFiRSSI);
  } else {
    latestWiFiRSSI = 0;
    latestWiFiPercent = 0;
    latestWiFiQuality = "access point";
  }

  latestBatteryVoltage = readBatteryVoltage();
  latestBatteryPercent = batteryPercentFromVoltage(latestBatteryVoltage);
  // BME280
  if (bmeOK) {
    latestTemperature = bme.readTemperature();
    latestHumidity = bme.readHumidity();
    latestPressure = bme.readPressure() / 100.0F;
    latestBmeOK = true;
  } else {
    latestBmeOK = false;
  }

  // BH1750
  if (bh1750OK) {
    latestLux = lightMeter.readLightLevel();
    latestBh1750OK = true;
  } else {
    latestBh1750OK = false;
  }

  // INA219 load
  if (inaLoadOK) {
    latestLoadVoltage = inaLoad.getBusVoltage_V();
    latestLoadCurrentMA = inaLoad.getCurrent_mA();
    latestLoadPowerMW = inaLoad.getPower_mW();
    latestLoadOK = true;
  } else {
    latestLoadOK = false;
  }

  // INA219 solar
  if (inaSolarOK) {
    latestSolarVoltage = inaSolar.getBusVoltage_V();
    latestSolarCurrentMA = inaSolar.getCurrent_mA();
    latestSolarPowerMW = inaSolar.getPower_mW();
    latestSolarOK = true;
  } else {
    latestSolarOK = false;
  }

  // Net battery current
  if (latestLoadOK && latestSolarOK) {
    latestNetCurrentMA = latestSolarCurrentMA - latestLoadCurrentMA;

    if (latestNetCurrentMA > 10) {
      latestBatteryState = "charging";
    } else if (latestNetCurrentMA < -10) {
      latestBatteryState = "discharging";
    } else {
      latestBatteryState = "balanced";
    }
  } else {
    latestNetCurrentMA = 0;
    latestBatteryState = "unknown";
  }

  // Wind speed
  uint16_t windSpeedRaw = 0;

  latestWindSpeedOK = readWindSpeed(
    latestWindMS,
    latestWindKPH,
    latestWindMPH,
    windSpeedRaw
  );

  if (latestWindSpeedOK) {
    addWindSample(latestWindMS);
    resetGustIfNeeded();

    latestWindAvgMS = getAverageWindMS();
    latestWindAvgKPH = latestWindAvgMS * 3.6;
    latestWindAvgMPH = latestWindAvgMS * 2.23694;

    latestWindGustMS = windGustMS;
    latestWindGustKPH = windGustKPH;
    latestWindGustMPH = windGustMPH;
  }

  delay(100);

  // Wind direction
  latestWindDirOK = readWindDirection(
    latestWindDirRaw,
    latestWindDirName,
    latestWindDirDegrees
  );
}

//------Web Handlers ---------------
void handleHeltecOn() {
  heltecPowerOn = true;
  digitalWrite(HELTEC_POWER_PIN, HIGH);
  server.send(200, "application/json", "{\"heltec_power\":\"on\"}");
}

void handleHeltecOff() {
  heltecPowerOn = false;
  digitalWrite(HELTEC_POWER_PIN, LOW);
  server.send(200, "application/json", "{\"heltec_power\":\"off\"}");
}

float readBatteryVoltage() {
  // Take a small average to reduce ADC noise
  const int samples = 20;
  long total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogRead(BATTERY_ADC_PIN);
    delay(2);
  }

  float raw = total / (float)samples;
  float adcVoltage = (raw / ADC_MAX_READING) * ADC_REF_VOLTAGE;

  return adcVoltage * BATTERY_DIVIDER_RATIO;
}

int batteryPercentFromVoltage(float voltage) {
  float percent = ((voltage - BATTERY_EMPTY_VOLTAGE) /
                   (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE)) * 100.0;

  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  return (int)(percent + 0.5);
}

int wifiPercentFromRSSI(int rssi) {
  if (rssi <= -90) return 0;
  if (rssi >= -30) return 100;

  return 2 * (rssi + 90);
}

String wifiQualityFromRSSI(int rssi) {
  if (rssi >= -50) return "excellent";
  if (rssi >= -60) return "good";
  if (rssi >= -70) return "fair";
  if (rssi >= -80) return "weak";
  return "poor";
}

// ----------------------------------------------------
// Setup
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32 Weather Station - Basic Sketch");
  Serial.println("====================================");

  pinMode(HELTEC_POWER_PIN, OUTPUT);
  digitalWrite(HELTEC_POWER_PIN, HELTEC_POWER_DEFAULT);

  Wire.begin(I2C_SDA, I2C_SCL);

  scanI2C();

  setupBME280();
  setupBH1750();
  setupINA219();
  setupWindSensors();

  pinMode(BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);

  Serial.println();
  
  setupWiFiAndServer();
  updateWeatherData();
  Serial.println("Setup complete.");
  lastGustReset = millis();

}

// ----------------------------------------------------
// Main loop
// ----------------------------------------------------
void loop() {
  server.handleClient();

  if (millis() - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdate = millis();

    updateWeatherData();

    Serial.println();
    Serial.println("========== WEATHER DATA ==========");

    Serial.print("Temperature: ");
    Serial.print(latestTemperature, 1);
    Serial.println(" C");

    Serial.print("Humidity:    ");
    Serial.print(latestHumidity, 1);
    Serial.println(" %");

    Serial.print("Pressure:    ");
    Serial.print(latestPressure, 1);
    Serial.println(" hPa");

    Serial.print("Light:       ");
    Serial.print(latestLux, 1);
    Serial.println(" lux");

    Serial.print("Load:        ");
    Serial.print(latestLoadVoltage, 3);
    Serial.print(" V | ");
    Serial.print(latestLoadCurrentMA, 2);
    Serial.print(" mA | ");
    Serial.print(latestLoadPowerMW, 2);
    Serial.println(" mW");

    Serial.print("Solar:       ");
    Serial.print(latestSolarVoltage, 3);
    Serial.print(" V | ");
    Serial.print(latestSolarCurrentMA, 2);
    Serial.print(" mA | ");
    Serial.print(latestSolarPowerMW, 2);
    Serial.println(" mW");

    Serial.print("Battery:     ");
    Serial.print(latestBatteryState);
    Serial.print(" | net ");
    Serial.print(latestNetCurrentMA, 2);
    Serial.println(" mA");

    Serial.print("Wind speed:  ");
    Serial.print(latestWindMS, 1);
    Serial.print(" m/s | ");
    Serial.print(latestWindKPH, 1);
    Serial.print(" km/h | ");
    Serial.print(latestWindMPH, 1);
    Serial.println(" mph");

    Serial.print("Wind avg:    ");
    Serial.print(latestWindAvgMS, 1);
    Serial.print(" m/s | ");
    Serial.print(latestWindAvgMPH, 1);
    Serial.println(" mph");

    Serial.print("Wind gust:   ");
    Serial.print(latestWindGustMS, 1);
    Serial.print(" m/s | ");
    Serial.print(latestWindGustMPH, 1);
    Serial.println(" mph");

    Serial.print("Direction:   ");
    Serial.print(latestWindDirName);
    Serial.print(" | ");
    Serial.print(latestWindDirDegrees, 1);
    Serial.println(" degrees");

    Serial.print("WiFi:        ");
    Serial.print(latestWiFiRSSI);
    Serial.print(" dBm | ");
    Serial.print(latestWiFiPercent);
    Serial.print(" % | ");
    Serial.println(latestWiFiQuality);

    Serial.println("==================================");
  }
}