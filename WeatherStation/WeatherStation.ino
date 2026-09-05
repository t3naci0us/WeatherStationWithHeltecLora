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
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <ArduinoOTA.h>
#include "secrets.h"

// ----------------------------------------------------
// WIFI SETTINGS
// ----------------------------------------------------
const char* WIFI_SSID = WIFI_SSID;
const char* WIFI_PASSWORD = WIFI_PASSWORD;

WebServer server(80);
String latestOTAHostname = "";
// ----------------------------------------------------
// OTA SETTINGS
// ----------------------------------------------------
const char* OTA_HOSTNAME = "weatherstation";
const char* OTA_PASSWORD = OTA_PASSWORD;

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

// ----------------------------------------------------
// SENSOR LAST-SEEN STATUS
// ----------------------------------------------------
unsigned long lastSeenBME280 = 0;
unsigned long lastSeenBH1750 = 0;
unsigned long lastSeenINALoad = 0;
unsigned long lastSeenINASolar = 0;
unsigned long lastSeenWindSpeed = 0;
unsigned long lastSeenWindDirection = 0;
unsigned long lastSeenSD = 0;

const unsigned long SENSOR_STALE_TIME = 10000; // 10 seconds

// ----------------------------------------------------
// SD LOGGING
// ----------------------------------------------------
unsigned long lastSDLog = 0;
const unsigned long SD_LOG_INTERVAL = 60000; // 60 seconds

// ----------------------------------------------------
// WIFI FALLBACK / RECONNECT SETTINGS
// ----------------------------------------------------
const char* AP_SSID = "WeatherStation";
const char* AP_PASSWORD = "REMOVED_OTA_PASSWORD123";

bool fallbackAPActive = false;
bool wifiWasConnected = false;

unsigned long lastWiFiCheck = 0;
unsigned long wifiLostAt = 0;
unsigned long lastReconnectAttempt = 0;

int wifiReconnectCount = 0;

const unsigned long WIFI_CHECK_INTERVAL = 5000;       // check every 5 sec
const unsigned long WIFI_RECONNECT_INTERVAL = 15000;  // retry every 15 sec
const unsigned long WIFI_AP_FALLBACK_AFTER = 30000;   // AP after 30 sec offline

//----Power switch-----
bool heltecPowerOn = false;

//-----------------------------------------------------
//System Health Globals
//-----------------------------------------------------
String latestUptime = "0s";
uint32_t latestFreeHeap = 0;
String latestResetReason = "";
String latestWiFiMode = "unknown";

// ----------------------------------------------------
// SD CARD
// ----------------------------------------------------
#define SD_CS   5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18

bool sdOK = false;

// ----------------------------------------------------
// BATTERY ADC
// ----------------------------------------------------
#define BATTERY_ADC_PIN 34

// Your divider is 100k / 100k, so ADC voltage is half battery voltage.
#define BATTERY_DIVIDER_RATIO 2.0

// ESP32 ADC reference is not perfect, so this may need calibration later.
#define ADC_REF_VOLTAGE 3.3
#define ADC_MAX_READING 4095.0
#define BATTERY_CALIBRATION 1.093

// 1S Li-ion rough voltage range
#define BATTERY_FULL_VOLTAGE 4.1
#define BATTERY_EMPTY_VOLTAGE 3.20

// ----------------------------------------------------
// LOW VOLTAGE PROTECTION
// ----------------------------------------------------
#define BATTERY_WARN_VOLTAGE      3.45
#define BATTERY_HELTEC_OFF_VOLTAGE 3.35
#define BATTERY_CRITICAL_VOLTAGE  3.20
#define BATTERY_RECOVER_VOLTAGE   3.70

bool lowVoltageLockout = false;
bool heltecRequestedOn = false;

String latestLowVoltageStatus = "normal";
String latestHeltecProtection = "allowed";

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
// REBOOT GLOBALS
// ----------------------------------------------------
bool rebootRequested = false;
unsigned long rebootRequestedAt = 0;
const unsigned long REBOOT_DELAY_MS = 2000;

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

void applyHeltecPower(bool on) {
  heltecPowerOn = on;
  digitalWrite(HELTEC_POWER_PIN, on ? HIGH : LOW);
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

void handleReboot() {
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"ESP32 rebooting\"}");

  rebootRequested = true;
  rebootRequestedAt = millis();

  Serial.println("Safe reboot requested from dashboard.");
}

//------------------------------------
// History Summary
//------------------------------------

#define HISTORY_CHART_POINTS 80

struct HistorySummary {
  String rangeLabel = "unknown";
  int rows = 0;

  float tempMin = 9999;
  float tempMax = -9999;
  float tempSum = 0;

  float humidityMin = 9999;
  float humidityMax = -9999;
  float humiditySum = 0;

  float pressureStart = 0;
  float pressureEnd = 0;
  bool pressureStarted = false;

  float luxMax = 0;
  float solarPowerMax = 0;
  float windMaxKPH = 0;
  float gustMaxMS = 0;
  float batteryMin = 9999;
  float batterySum = 0;
  float wifiMin = 9999;
  float wifiMax = -9999;
  float wifiSum = 0;
  float netCurrentSum = 0;

  int dirCounts[16] = {0};
};

struct ChartData {
  float temp[HISTORY_CHART_POINTS];
  float humidity[HISTORY_CHART_POINTS];
  float pressure[HISTORY_CHART_POINTS];
  float solar[HISTORY_CHART_POINTS];
  float battery[HISTORY_CHART_POINTS];
  float wind[HISTORY_CHART_POINTS];
  float wifi[HISTORY_CHART_POINTS];

  int count = 0;
};

//------------------------------------
// CSV Helpers
//------------------------------------
String getCSVField(const String &line, int index) {
  int start = 0;
  int currentIndex = 0;

  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line.charAt(i) == ',') {
      if (currentIndex == index) {
        return line.substring(start, i);
      }

      start = i + 1;
      currentIndex++;
    }
  }

  return "";
}

time_t parseTimestamp(const String &stamp) {
  // Expected format: YYYY-MM-DD HH:MM:SS
  if (stamp.length() < 19) return 0;

  struct tm t;
  memset(&t, 0, sizeof(t));

  t.tm_year = stamp.substring(0, 4).toInt() - 1900;
  t.tm_mon  = stamp.substring(5, 7).toInt() - 1;
  t.tm_mday = stamp.substring(8, 10).toInt();
  t.tm_hour = stamp.substring(11, 13).toInt();
  t.tm_min  = stamp.substring(14, 16).toInt();
  t.tm_sec  = stamp.substring(17, 19).toInt();

  return mktime(&t);
}

String dominantDirectionsJson(int counts[16]) {
  const char* names[16] = {
    "N","NNE","NE","ENE","E","ESE","SE","SSE",
    "S","SSW","SW","WSW","W","WNW","NW","NNW"
  };

  String json = "[";
  bool first = true;

  for (int i = 0; i < 16; i++) {
    if (counts[i] > 0) {
      if (!first) json += ",";
      json += "{\"dir\":\"" + String(names[i]) + "\",\"count\":" + String(counts[i]) + "}";
      first = false;
    }
  }

  json += "]";
  return json;
}

String floatArrayJson(float values[], int count, int decimals) {
  String json = "[";

  for (int i = 0; i < count; i++) {
    if (i > 0) json += ",";
    json += String(values[i], decimals);
  }

  json += "]";
  return json;
}

//------------------------------------
// History Endpoint
//------------------------------------
void handleHistory() {
  if (!sdOK || !SD.exists("/weather.csv")) {
    server.send(404, "application/json", "{\"ok\":false,\"message\":\"No weather log found\"}");
    return;
  }

  String range = server.arg("range");
  if (range == "") range = "24h";

  unsigned long rangeSeconds = 24UL * 60UL * 60UL;

  if (range == "7d") {
    rangeSeconds = 7UL * 24UL * 60UL * 60UL;
  } else if (range == "30d") {
    rangeSeconds = 30UL * 24UL * 60UL * 60UL;
  }

time_t nowTime = 0;
bool haveTime = false;

struct tm nowInfo;

if (getLocalTime(&nowInfo, 1000)) {
  nowTime = mktime(&nowInfo);
  haveTime = true;
}

  File file = SD.open("/weather.csv", FILE_READ);

  if (!file) {
    server.send(500, "application/json", "{\"ok\":false,\"message\":\"Could not open weather.csv\"}");
    return;
  }

  HistorySummary s;
  ChartData chart;

  s.rangeLabel = range;

  bool headerSkipped = false;

  int validRowsSeen = 0;
  int chartStride = 1;

  if (range == "24h") chartStride = 1;
  if (range == "7d") chartStride = 5;
  if (range == "30d") chartStride = 20;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) continue;

    if (!headerSkipped) {
      headerSkipped = true;
      continue;
    }

    String timestamp = getCSVField(line, 0);

    if (haveTime) {
      time_t rowTime = parseTimestamp(timestamp);
      if (rowTime == 0) continue;

      double age = difftime(nowTime, rowTime);

    // Allow up to 2 hours future tolerance.
    // This protects against UTC/BST/DST offset oddities.
    //if (age < -7200 || age > rangeSeconds) {
    //  rowInRange = false;
    //}
    }

    float temp = getCSVField(line, 1).toFloat();
    float hum = getCSVField(line, 2).toFloat();
    float pressure = getCSVField(line, 3).toFloat();
    float lux = getCSVField(line, 4).toFloat();
    float battV = getCSVField(line, 5).toFloat();
    float battPercent = getCSVField(line, 6).toFloat();
    float solarPower = getCSVField(line, 12).toFloat();
    float netCurrent = getCSVField(line, 13).toFloat();
    float windKPH = getCSVField(line, 16).toFloat();
    float gustMS = getCSVField(line, 19).toFloat();
    String windDir = getCSVField(line, 20);
    float wifiPercent = getCSVField(line, 23).toFloat();

    s.rows++;
    validRowsSeen++;

    if (temp < s.tempMin) s.tempMin = temp;
    if (temp > s.tempMax) s.tempMax = temp;
    s.tempSum += temp;

    if (hum < s.humidityMin) s.humidityMin = hum;
    if (hum > s.humidityMax) s.humidityMax = hum;
    s.humiditySum += hum;

    if (!s.pressureStarted) {
      s.pressureStart = pressure;
      s.pressureStarted = true;
    }
    s.pressureEnd = pressure;

    if (lux > s.luxMax) s.luxMax = lux;
    if (solarPower > s.solarPowerMax) s.solarPowerMax = solarPower;
    if (windKPH > s.windMaxKPH) s.windMaxKPH = windKPH;
    if (gustMS > s.gustMaxMS) s.gustMaxMS = gustMS;
    if (battV < s.batteryMin) s.batteryMin = battV;

    s.batterySum += battPercent;
    s.netCurrentSum += netCurrent;

    if (wifiPercent < s.wifiMin) s.wifiMin = wifiPercent;
    if (wifiPercent > s.wifiMax) s.wifiMax = wifiPercent;
    s.wifiSum += wifiPercent;

    const char* names[16] = {
      "N","NNE","NE","ENE","E","ESE","SE","SSE",
      "S","SSW","SW","WSW","W","WNW","NW","NNW"
    };

    for (int i = 0; i < 16; i++) {
      if (windDir == names[i]) {
        s.dirCounts[i]++;
        break;
      }
    }

    // Downsample chart points so JSON stays small.
    if ((validRowsSeen % chartStride == 0) && chart.count < HISTORY_CHART_POINTS) {
      chart.temp[chart.count] = temp;
      chart.humidity[chart.count] = hum;
      chart.pressure[chart.count] = pressure;
      chart.solar[chart.count] = solarPower;
      chart.battery[chart.count] = battPercent;
      chart.wind[chart.count] = windKPH;
      chart.wifi[chart.count] = wifiPercent;
      chart.count++;
    }

    if (s.rows % 50 == 0) {
      server.handleClient();
      delay(1);
    }
  }

  file.close();

  if (s.rows == 0) {
    server.send(200, "application/json", "{\"ok\":false,\"message\":\"No rows in selected range\"}");
    return;
  }

  float tempAvg = s.tempSum / s.rows;
  float humAvg = s.humiditySum / s.rows;
  float battAvg = s.batterySum / s.rows;
  float wifiAvg = s.wifiSum / s.rows;
  float netAvg = s.netCurrentSum / s.rows;

  String pressureTrend = "steady";
  if (s.pressureEnd > s.pressureStart + 1.0) pressureTrend = "rising";
  if (s.pressureEnd < s.pressureStart - 1.0) pressureTrend = "falling";

  String json = "{";
  json += "\"ok\":true,";
  json += "\"range\":\"" + range + "\",";
  json += "\"rows\":" + String(s.rows) + ",";

  json += "\"temp_min\":" + String(s.tempMin, 2) + ",";
  json += "\"temp_max\":" + String(s.tempMax, 2) + ",";
  json += "\"temp_avg\":" + String(tempAvg, 2) + ",";

  json += "\"humidity_min\":" + String(s.humidityMin, 2) + ",";
  json += "\"humidity_max\":" + String(s.humidityMax, 2) + ",";
  json += "\"humidity_avg\":" + String(humAvg, 2) + ",";

  json += "\"pressure_start\":" + String(s.pressureStart, 2) + ",";
  json += "\"pressure_end\":" + String(s.pressureEnd, 2) + ",";
  json += "\"pressure_trend\":\"" + pressureTrend + "\",";

  json += "\"lux_max\":" + String(s.luxMax, 2) + ",";
  json += "\"solar_power_max\":" + String(s.solarPowerMax, 2) + ",";
  json += "\"wind_max_kph\":" + String(s.windMaxKPH, 2) + ",";
  json += "\"gust_max_ms\":" + String(s.gustMaxMS, 2) + ",";

  json += "\"battery_min\":" + String(s.batteryMin, 3) + ",";
  json += "\"battery_avg\":" + String(battAvg, 2) + ",";

  json += "\"wifi_min\":" + String(s.wifiMin, 1) + ",";
  json += "\"wifi_max\":" + String(s.wifiMax, 1) + ",";
  json += "\"wifi_avg\":" + String(wifiAvg, 1) + ",";

  json += "\"net_current_avg\":" + String(netAvg, 2) + ",";
  json += "\"direction_counts\":" + dominantDirectionsJson(s.dirCounts) + ",";

  json += "\"chart_temp\":" + floatArrayJson(chart.temp, chart.count, 2) + ",";
  json += "\"chart_humidity\":" + floatArrayJson(chart.humidity, chart.count, 1) + ",";
  json += "\"chart_pressure\":" + floatArrayJson(chart.pressure, chart.count, 1) + ",";
  json += "\"chart_solar\":" + floatArrayJson(chart.solar, chart.count, 0) + ",";
  json += "\"chart_battery\":" + floatArrayJson(chart.battery, chart.count, 0) + ",";
  json += "\"chart_wind\":" + floatArrayJson(chart.wind, chart.count, 1) + ",";
  json += "\"chart_wifi\":" + floatArrayJson(chart.wifi, chart.count, 0);

  json += "}";

  server.send(200, "application/json", json);
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

    .btn.link {
      display: inline-block;
      text-decoration: none;
      background: #8be9fd;
    }

    .btn.off {
      background: #ff7b7b;
    }
    .small {
      font-size: 14px;
      color: #b8c4d6;
      line-height: 1.5;
    }

    .battery-shell {
      width: 100%;
      height: 24px;
      border: 2px solid #8be9fd;
      border-radius: 8px;
      overflow: hidden;
      background: #0d111a;
      margin: 10px 0;
      position: relative;
    }

    .battery-fill {
      height: 100%;
      width: 0%;
      background: #6dff9b;
      transition: width 0.4s ease, background 0.4s ease;
    }

    .battery-warn {
      background: #ffd36d;
    }

    .battery-low {
      background: #ff7b7b;
    }

    .battery-critical {
      background: #ff3b3b;
    }

    .danger-note {
      color: #ff7b7b;
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
      <div class="small">Reconnects: <span id="wifi_reconnects">--</span></div>
      <div class="small">Fallback AP: <span id="fallback_ap">--</span></div>
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

      <div class="value">
        <span id="battery_percent">--</span><span class="unit">%</span>
      </div>

      <div class="battery-shell">
        <div id="battery_fill" class="battery-fill"></div>
      </div>

      <div class="small">Voltage: <span id="battery_voltage">--</span> V</div>
      <div class="small">Net current: <span id="battery_net_current">--</span> mA</div>
      <div class="status">State: <span id="battery_state_2">--</span></div>
      <div class="small">LV status: <span id="battery_lv_status">--</span></div>
    </div>

    <div class="card">
      <h2>SD Log</h2>
      <div class="small">Download or clear the weather CSV log.</div>
      <a class="btn link" href="/download-log">Download CSV</a>
      <button class="btn off" onclick="clearLog()">Clear Log</button>
      <div class="status">Log status: <span id="log_action_status">ready</span></div>
    </div>

    <div class="card">
      <h2>Sensor Status</h2>
      <div class="small">BME280: <span id="status_bme280">--</span></div>
      <div class="small">BH1750: <span id="status_bh1750">--</span></div>
      <div class="small">Load INA219: <span id="status_ina_load">--</span></div>
      <div class="small">Solar INA219: <span id="status_ina_solar">--</span></div>
      <div class="small">Wind Speed: <span id="status_wind_speed">--</span></div>
      <div class="small">Wind Direction: <span id="status_wind_dir">--</span></div>
      <div class="small">SD Logging: <span id="status_sd">--</span></div>
    </div>
    <div class="card">
      <h2>System Health</h2>
      <div class="small">Uptime: <span id="uptime">--</span></div>
      <div class="small">Free heap: <span id="free_heap">--</span> bytes</div>
      <div class="small">Reset reason: <span id="reset_reason">--</span></div>
      <div class="small">Wi-Fi mode: <span id="wifi_mode">--</span></div>
    </div>
    <div class="card">
      <h2>OTA Update</h2>
      <div class="small">Hostname: <span id="ota_hostname">--</span></div>
      <div class="small">Arduino IDE network upload should show this device once connected.</div>
    </div>
    <div class="card">
      <h2>Low Voltage Protection</h2>
      <div class="small">Status: <span id="low_voltage_status">--</span></div>
      <div class="small">Lockout: <span id="low_voltage_lockout">--</span></div>
      <div class="small">Heltec requested: <span id="heltec_requested">--</span></div>
      <div class="small">Protection: <span id="heltec_protection">--</span></div>
      <div class="small">Heltec actual: <span id="heltec_power_2">--</span></div>
    </div>

    <div class="card">
      <h2>System Control</h2>
      <div class="small">Restart the ESP32 weather station safely.</div>
      <button class="btn off" onclick="if(confirm('Reboot ESP32 now?')) fetch('/reboot')">
  Reboot ESP32
</button>
      <div class="status">Action: <span id="system_action_status">ready</span></div>
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
    document.getElementById('wifi_reconnects').textContent = d.wifi_reconnects;
    document.getElementById('fallback_ap').textContent = d.fallback_ap;

    document.getElementById('battery_voltage').textContent = d.battery_voltage.toFixed(3);
    document.getElementById('battery_percent').textContent = d.battery_percent;
    document.getElementById('battery_state_2').textContent = d.battery_state;
    document.getElementById('battery_net_current').textContent = d.net_current.toFixed(2);
    document.getElementById('battery_lv_status').textContent = d.low_voltage_status;

    const batteryFill = document.getElementById('battery_fill');
    batteryFill.style.width = d.battery_percent + '%';

    batteryFill.className = 'battery-fill';

    if (d.low_voltage_status === 'critical') {
      batteryFill.classList.add('battery-critical');
    } else if (d.low_voltage_status === 'heltec off') {
      batteryFill.classList.add('battery-low');
    } else if (d.low_voltage_status === 'warning') {
      batteryFill.classList.add('battery-warn');
    }

    document.getElementById('status_bme280').textContent = d.status_bme280;
    document.getElementById('status_bh1750').textContent = d.status_bh1750;
    document.getElementById('status_ina_load').textContent = d.status_ina_load;
    document.getElementById('status_ina_solar').textContent = d.status_ina_solar;
    document.getElementById('status_wind_speed').textContent = d.status_wind_speed;
    document.getElementById('status_wind_dir').textContent = d.status_wind_dir;
    document.getElementById('status_sd').textContent = d.status_sd;

    document.getElementById('uptime').textContent = d.uptime;
    document.getElementById('free_heap').textContent = d.free_heap;
    document.getElementById('reset_reason').textContent = d.reset_reason;
    document.getElementById('wifi_mode').textContent = d.wifi_mode;

    const battery = document.getElementById('battery_state');
    battery.textContent = d.battery_state;
    battery.className = d.battery_state;

    document.getElementById('ota_hostname').textContent = d.ota_hostname;

    document.getElementById('heltec_power').textContent =
      d.heltec_power ? d.heltec_power.toUpperCase() : 'UNKNOWN';

async function safeReboot() {
  const sure = confirm('Reboot the ESP32 weather station now?');

  if (!sure) {
    return;
  }

  try {
    const res = await fetch('/reboot');
    const d = await res.json();

    const status = document.getElementById('system_action_status');

    if (status) {
      status.textContent = 'rebooting...';
    }

    setTimeout(() => {
      if (status) {
        status.textContent = 'reboot sent - reconnect in a few seconds';
      }
    }, 1000);

  } catch (e) {
    const status = document.getElementById('system_action_status');

    if (status) {
      status.textContent = 'reboot failed';
    } else {
      alert('Reboot command failed.');
    }
  }
}

    document.getElementById('low_voltage_status').textContent = d.low_voltage_status;
    document.getElementById('low_voltage_lockout').textContent = d.low_voltage_lockout;
    document.getElementById('heltec_requested').textContent = d.heltec_requested;
    document.getElementById('heltec_protection').textContent = d.heltec_protection;
    document.getElementById('heltec_power_2').textContent =
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
async function clearLog() {
  const sure = confirm('Clear the weather log? This cannot be undone.');

  if (!sure) {
    return;
  }

  try {
    const res = await fetch('/clear-log');
    const d = await res.json();

    document.getElementById('log_action_status').textContent =
      d.ok ? 'log cleared' : d.message;

    updateData();
  } catch (e) {
    document.getElementById('log_action_status').textContent = 'clear failed';
  }
}
updateData();
setInterval(updateData, 2000);
</script>
</body>
</html>
)rawliteral";

//---------------------------------------
//Report Page HTML
//---------------------------------------
const char REPORT_PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HandiWorx Weather Dashboard</title>

<style>
:root {
  --bg0: #030914;
  --bg1: #061428;
  --panel: rgba(12, 29, 52, 0.86);
  --panel2: rgba(18, 38, 66, 0.92);
  --line: rgba(116, 201, 255, 0.22);
  --line2: rgba(116, 201, 255, 0.42);
  --text: #f5f9ff;
  --muted: #9fb3cf;
  --cyan: #70e8ff;
  --blue: #38a6ff;
  --orange: #ff8a23;
  --yellow: #ffd447;
  --green: #8dff73;
  --purple: #b68cff;
  --red: #ff6475;
  --shadow: rgba(0,0,0,0.38);
}

.mini-chart {
  width: 100%;
  height: 105px;
  margin-top: 12px;
  border: 1px solid rgba(116,201,255,0.16);
  border-radius: 14px;
  background:
    linear-gradient(rgba(255,255,255,0.035) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255,255,255,0.035) 1px, transparent 1px),
    radial-gradient(circle at 50% 0%, rgba(112,232,255,0.07), transparent 55%),
    rgba(0,0,0,0.18);
  background-size: 22px 22px, 22px 22px, auto, auto;
  overflow: hidden;
  position: relative;
}

.mini-chart svg {
  width: 100%;
  height: 100%;
  display: block;
}

.chart-line {
  fill: none;
  stroke: var(--cyan);
  stroke-width: 3.2;
  stroke-linecap: round;
  stroke-linejoin: round;
  filter: drop-shadow(0 0 6px rgba(112,232,255,0.85));
}

.chart-area {
  fill: rgba(112,232,255,0.16);
}

.chart-line.orange { stroke: var(--orange); }
.chart-line.green { stroke: var(--green); }
.chart-line.purple { stroke: var(--purple); }
.chart-line.yellow { stroke: var(--yellow); }
.chart-line.red { stroke: var(--red); }

.chart-area.orange { fill: rgba(255,138,35,0.16); }
.chart-area.green { fill: rgba(141,255,115,0.13); }
.chart-area.purple { fill: rgba(182,140,255,0.14); }
.chart-area.yellow { fill: rgba(255,212,71,0.15); }
.chart-area.red { fill: rgba(255,100,117,0.13); }

.chart-label {
  position: absolute;
  right: 8px;
  top: 6px;
  font-size: 11px;
  color: var(--muted);
  background: rgba(0,0,0,0.25);
  border: 1px solid rgba(116,201,255,0.12);
  border-radius: 999px;
  padding: 4px 7px;
}

.mini-chart svg {
  width: 100%;
  height: 100%;
  display: block;
}

.condition-banner {
  display: grid;
  grid-template-columns: auto 1fr auto;
  gap: 16px;
  align-items: center;
  border: 1px solid var(--line);
  background:
    radial-gradient(circle at left, rgba(112,232,255,0.12), transparent 38%),
    linear-gradient(135deg, rgba(11,29,54,0.94), rgba(5,14,28,0.94));
  border-radius: 24px;
  padding: 18px;
  margin: 14px 0;
  box-shadow: 0 16px 36px rgba(0,0,0,0.28);
}

.condition-emoji {
  font-size: 58px;
  filter: drop-shadow(0 0 12px rgba(112,232,255,0.35));
}

.condition-title {
  font-size: clamp(24px, 5vw, 42px);
  font-weight: 900;
  line-height: 1;
}

.condition-sub {
  color: var(--muted);
  margin-top: 6px;
}

.condition-now {
  text-align: right;
  font-size: 28px;
  font-weight: 900;
  color: var(--cyan);
}

.status-chip-row {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  margin-top: 12px;
}

.status-chip {
  border: 1px solid rgba(116,201,255,0.22);
  background: rgba(255,255,255,0.055);
  border-radius: 999px;
  padding: 7px 10px;
  color: var(--muted);
  font-size: 13px;
}

@media (max-width: 620px) {
  .condition-banner {
    grid-template-columns: 1fr;
    text-align: center;
  }

  .condition-now {
    text-align: center;
  }
}

.chart-line {
  fill: none;
  stroke: var(--cyan);
  stroke-width: 3;
  stroke-linecap: round;
  stroke-linejoin: round;
  filter: drop-shadow(0 0 5px rgba(112,232,255,0.65));
}

.chart-line.orange { stroke: var(--orange); }
.chart-line.green { stroke: var(--green); }
.chart-line.purple { stroke: var(--purple); }
.chart-line.yellow { stroke: var(--yellow); }
.chart-line.red { stroke: var(--red); }

* {
  box-sizing: border-box;
}

body {
  margin: 0;
  color: var(--text);
  font-family: Arial, Helvetica, sans-serif;
  background:
    radial-gradient(circle at 10% 4%, rgba(82, 180, 255, 0.20), transparent 28%),
    radial-gradient(circle at 88% 2%, rgba(170, 91, 255, 0.14), transparent 26%),
    radial-gradient(circle at 50% 100%, rgba(0, 255, 200, 0.06), transparent 36%),
    linear-gradient(180deg, var(--bg1), var(--bg0));
  min-height: 100vh;
}

body::before {
  content: "";
  position: fixed;
  inset: 0;
  pointer-events: none;
  background-image:
    linear-gradient(rgba(112,232,255,0.035) 1px, transparent 1px),
    linear-gradient(90deg, rgba(112,232,255,0.035) 1px, transparent 1px);
  background-size: 42px 42px;
  mask-image: linear-gradient(to bottom, black, transparent 86%);
}

.wrapper {
  width: min(1280px, 100%);
  margin: auto;
  padding: 16px;
}

.hero {
  position: relative;
  overflow: hidden;
  border: 1px solid var(--line);
  border-radius: 28px;
  padding: 24px 18px 20px;
  margin-bottom: 14px;
  background:
    linear-gradient(135deg, rgba(11,29,54,0.92), rgba(5,14,28,0.92)),
    radial-gradient(circle at right, rgba(112,232,255,0.16), transparent 40%);
  box-shadow: 0 18px 40px var(--shadow), inset 0 0 28px rgba(112,232,255,0.04);
}

.hero-top {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 14px;
  flex-wrap: wrap;
}

.logo-block h1 {
  margin: 0;
  font-size: clamp(36px, 7vw, 76px);
  line-height: 0.92;
  letter-spacing: -3px;
  text-shadow: 0 0 24px rgba(112,232,255,0.20);
}

.logo-block .sub {
  margin-top: 10px;
  color: var(--cyan);
  font-size: clamp(15px, 2.5vw, 20px);
  font-weight: 700;
  letter-spacing: 1px;
}

.weather-icon {
  font-size: clamp(54px, 12vw, 110px);
  filter: drop-shadow(0 0 18px rgba(112,232,255,0.35));
}

.nav {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  margin-top: 18px;
}

.nav button,
.nav a {
  border: 1px solid var(--line2);
  border-radius: 999px;
  color: var(--text);
  background: rgba(20, 45, 76, 0.72);
  padding: 11px 16px;
  font-weight: 800;
  letter-spacing: 0.4px;
  text-decoration: none;
  cursor: pointer;
  box-shadow: inset 0 0 16px rgba(112,232,255,0.05);
}

.nav button.active {
  background: linear-gradient(135deg, var(--cyan), var(--blue));
  color: #03101f;
  border-color: transparent;
}

.top-stats {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
  gap: 12px;
  margin: 14px 0;
}

.pill {
  border: 1px solid var(--line);
  background: rgba(8, 22, 42, 0.78);
  border-radius: 18px;
  padding: 14px;
  text-align: center;
  box-shadow: 0 10px 26px rgba(0,0,0,0.22);
}

.pill strong {
  display: block;
  color: var(--cyan);
  font-size: 24px;
}

.pill span {
  color: var(--muted);
  font-size: 13px;
}

.section {
  border: 1px solid var(--line);
  background: rgba(5, 16, 32, 0.70);
  border-radius: 24px;
  padding: 16px;
  margin-top: 14px;
  box-shadow: 0 16px 38px rgba(0,0,0,0.30), inset 0 0 32px rgba(112,232,255,0.035);
}

.section-title {
  display: flex;
  align-items: center;
  gap: 10px;
  font-size: 23px;
  font-weight: 900;
  letter-spacing: 0.8px;
  margin-bottom: 14px;
}

.grid3 {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(245px, 1fr));
  gap: 12px;
}

.grid2 {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(290px, 1fr));
  gap: 12px;
}

.card {
  position: relative;
  overflow: hidden;
  border: 1px solid var(--line);
  border-radius: 18px;
  padding: 16px;
  min-height: 150px;
  background:
    linear-gradient(180deg, rgba(20, 40, 70, 0.96), rgba(9, 24, 44, 0.96));
  box-shadow: 0 12px 28px rgba(0,0,0,0.24);
}

.card::after {
  content: "";
  position: absolute;
  inset: auto -30% -40% auto;
  width: 190px;
  height: 190px;
  background: radial-gradient(circle, rgba(112,232,255,0.09), transparent 65%);
  pointer-events: none;
}

.card h2 {
  margin: 0 0 10px;
  font-size: 18px;
  color: var(--cyan);
  text-transform: uppercase;
  letter-spacing: 1.3px;
}

.big {
  font-size: clamp(34px, 8vw, 52px);
  font-weight: 900;
  line-height: 1;
  margin: 8px 0 12px;
}

.unit {
  color: var(--muted);
  font-size: 18px;
  margin-left: 4px;
}

.small {
  color: var(--muted);
  font-size: 15px;
  line-height: 1.6;
}

.stat-line {
  display: flex;
  justify-content: space-between;
  gap: 10px;
  border-top: 1px solid rgba(116,201,255,0.12);
  padding-top: 8px;
  margin-top: 8px;
}

.orange { color: var(--orange); }
.green { color: var(--green); }
.cyan { color: var(--cyan); }
.purple { color: var(--purple); }
.red { color: var(--red); }
.yellow { color: var(--yellow); }

.meter {
  width: 100%;
  height: 14px;
  border-radius: 999px;
  background: rgba(255,255,255,0.08);
  overflow: hidden;
  margin: 12px 0;
  border: 1px solid rgba(116,201,255,0.18);
}

.fill {
  height: 100%;
  width: 0%;
  background: linear-gradient(90deg, var(--blue), var(--cyan));
  border-radius: 999px;
  transition: width 0.4s ease;
}

.fill.battery {
  background: linear-gradient(90deg, var(--green), var(--cyan));
}

.fill.solar {
  background: linear-gradient(90deg, var(--orange), var(--yellow));
}

.fill.wifi {
  background: linear-gradient(90deg, var(--purple), var(--cyan));
}

.alert {
  margin-top: 10px;
  padding: 10px 12px;
  border-radius: 12px;
  background: rgba(255,255,255,0.06);
  color: var(--muted);
}

.takeaways {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(245px, 1fr));
  gap: 12px;
}

.takeaway {
  border: 1px solid var(--line);
  border-radius: 18px;
  padding: 16px;
  background: rgba(16, 37, 66, 0.88);
  color: #e6f1ff;
  min-height: 96px;
  display: flex;
  align-items: center;
  gap: 12px;
}

.takeaway .emoji {
  font-size: 34px;
}

.direction-list {
  display: grid;
  gap: 7px;
}

.dir-row {
  display: grid;
  grid-template-columns: 48px 1fr 54px;
  align-items: center;
  gap: 8px;
  color: var(--muted);
}

.dir-bar {
  height: 9px;
  background: rgba(255,255,255,0.08);
  border-radius: 999px;
  overflow: hidden;
}

.compass-wrap {
  display: flex;
  align-items: center;
  gap: 18px;
  flex-wrap: wrap;
}

.compass {
  width: 140px;
  height: 140px;
  border: 2px solid rgba(112,232,255,0.35);
  border-radius: 50%;
  position: relative;
  background:
    radial-gradient(circle, rgba(112,232,255,0.10), rgba(0,0,0,0.10) 55%, rgba(0,0,0,0.26)),
    conic-gradient(from 0deg, rgba(112,232,255,0.16), transparent, rgba(112,232,255,0.16));
  box-shadow: inset 0 0 24px rgba(112,232,255,0.08), 0 0 24px rgba(112,232,255,0.10);
}

.compass::before {
  content: "N";
  position: absolute;
  top: 6px;
  left: 50%;
  transform: translateX(-50%);
  color: var(--cyan);
  font-weight: 900;
}

.compass::after {
  content: "";
  position: absolute;
  inset: 18px;
  border: 1px solid rgba(255,255,255,0.08);
  border-radius: 50%;
}

.needle {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 5px;
  height: 52px;
  background: linear-gradient(var(--orange), var(--cyan));
  border-radius: 999px;
  transform-origin: 50% 95%;
  transform: translate(-50%, -95%) rotate(0deg);
  box-shadow: 0 0 10px rgba(112,232,255,0.7);
}

.compass-centre {
  position: absolute;
  width: 16px;
  height: 16px;
  background: var(--cyan);
  border-radius: 50%;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  box-shadow: 0 0 14px rgba(112,232,255,0.8);
}

.compass-info {
  flex: 1;
  min-width: 160px;
}

.dir-bar div {
  height: 100%;
  background: linear-gradient(90deg, var(--blue), var(--cyan));
}

.footer {
  text-align: center;
  color: var(--muted);
  padding: 24px 8px 12px;
  font-size: 13px;
}

@media (max-width: 620px) {
  .wrapper {
    padding: 10px;
  }

  .hero {
    border-radius: 20px;
    padding: 18px 14px;
  }

  .logo-block h1 {
    letter-spacing: -1px;
  }

  .weather-icon {
    display: none;
  }

  .section {
    border-radius: 18px;
    padding: 12px;
  }

  .card {
    border-radius: 16px;
    min-height: unset;
  }
}
</style>
</head>

<body>
<div class="wrapper">

  <header class="hero">
    <div class="hero-top">
      <div class="logo-block">
        <h1>Weather Log<br>Dashboard</h1>
        <div class="sub">HandiWorx Solar Weather Station</div>
      </div>
      <div class="weather-icon">🌦️</div>
    </div>

    <section class="condition-banner">
      <div class="condition-emoji" id="conditionEmoji">🌦️</div>

      <div>
        <div class="condition-title" id="conditionTitle">Live Weather Station</div>
        <div class="condition-sub" id="conditionSub">
          Reading current conditions from the ESP32 station.
        </div>

        <div class="status-chip-row">
          <div class="status-chip">Battery <strong id="chipBattery">--%</strong></div>
          <div class="status-chip">Solar <strong id="chipSolar">-- mW</strong></div>
          <div class="status-chip">Wind <strong id="chipWind">-- mph</strong></div>
          <div class="status-chip">Wi-Fi <strong id="chipWifi">--%</strong></div>
        </div>
      </div>

      <div class="condition-now">
        <span id="conditionTemp">--</span>°C
      </div>
    </section>
    
    <div class="nav">
      <button id="btn24h" onclick="setRange('24h')" class="active">24 Hours</button>
      <button id="btn7d" onclick="setRange('7d')">7 Days</button>
      <button id="btn30d" onclick="setRange('30d')">30 Days</button>
      <a href="/cards">Engineering View</a>
      <a href="/download-log">Download CSV</a>
      <button onclick="safeReboot()">Reboot ESP32</button>
    </div>
  </header>

  <div class="top-stats">
    <div class="pill"><strong id="samples">--</strong><span>samples in range</span></div>
    <div class="pill"><strong id="rangeLabel">24h</strong><span>selected range</span></div>
    <div class="pill"><strong id="liveBattery">--%</strong><span>battery now</span></div>
    <div class="pill"><strong id="liveWifi">--%</strong><span>Wi-Fi now</span></div>
  </div>

  <section class="section">
    <div class="section-title">🌤️ 1. Weather Snapshot</div>

    <div class="grid3">
      <div class="card">
        <h2 class="orange">Temperature</h2>
        <div class="big"><span id="tempNow">--</span><span class="unit">°C</span></div>
        <div class="mini-chart" id="chart_temp"></div>
        <div class="stat-line"><span>Average</span><strong><span id="tempAvg">--</span>°C</strong></div>
        <div class="stat-line"><span>Minimum</span><strong><span id="tempMin">--</span>°C</strong></div>
        <div class="stat-line"><span>Maximum</span><strong><span id="tempMax">--</span>°C</strong></div>
      </div>

      <div class="card">
        <h2 class="green">Humidity</h2>
        <div class="big"><span id="humNow">--</span><span class="unit">%</span></div>
        <div class="meter"><div id="humFill" class="fill"></div></div>
        <div class="mini-chart" id="chart_humidity"></div>
        <div class="stat-line"><span>Average</span><strong><span id="humAvg">--</span>%</strong></div>
        <div class="stat-line"><span>Range</span><strong><span id="humMin">--</span>% → <span id="humMax">--</span>%</strong></div>
      </div>

      <div class="card">
        <h2 class="cyan">Pressure</h2>
        <div class="big"><span id="pressureNow">--</span><span class="unit">hPa</span></div>
        <div class="mini-chart" id="chart_pressure"></div>
        <div class="stat-line"><span>Start</span><strong><span id="pressureStart">--</span> hPa</strong></div>
        <div class="stat-line"><span>End</span><strong><span id="pressureEnd">--</span> hPa</strong></div>
        <div class="alert">Trend: <strong id="pressureTrend">--</strong></div>
      </div>
    </div>
  </section>

  <section class="section">
    <div class="section-title">☀️ 2. Light & Power</div>

    <div class="grid2">
      <div class="card">
        <h2 class="yellow">Light & Solar</h2>
        <div class="big"><span id="solarNow">--</span><span class="unit">mW</span></div>
        <div class="meter"><div id="solarFill" class="fill solar"></div></div>
        <div class="mini-chart" id="chart_battery"></div>
        <div class="mini-chart" id="chart_solar"></div>
        <div class="stat-line"><span>Solar peak</span><strong><span id="solarPeak">--</span> mW</strong></div>
        <div class="stat-line"><span>Light now</span><strong><span id="luxNow">--</span> lux</strong></div>
        <div class="stat-line"><span>Peak light</span><strong><span id="luxPeak">--</span> lux</strong></div>
      </div>

      <div class="card">
        <h2 class="green">Battery</h2>
        <div class="big"><span id="batteryNow">--</span><span class="unit">%</span></div>
        <div class="meter"><div id="batteryFill" class="fill battery"></div></div>
        <div class="stat-line"><span>Average</span><strong><span id="batteryAvg">--</span>%</strong></div>
        <div class="stat-line"><span>Lowest voltage</span><strong><span id="batteryMin">--</span> V</strong></div>
        <div class="alert">Net current avg: <strong id="netAvg">--</strong> mA</div>
      </div>
    </div>
  </section>

  <section class="section">
    <div class="section-title">💨 3. Wind & Direction</div>

   <div class="card">
  <h2 class="cyan">Wind Now</h2>

  <div class="compass-wrap">
      <div class="compass">
        <div id="windNeedle" class="needle"></div>
        <div class="compass-centre"></div>
      </div>

      <div class="compass-info">
        <div class="big"><span id="windNow">--</span><span class="unit">mph</span></div>
        <div class="alert">Direction now: <strong id="dirNow">--</strong></div>
      </div>
    </div>

    <div class="mini-chart" id="chart_wind"></div>

    <div class="stat-line"><span>Max wind</span><strong><span id="windMax">--</span> km/h</strong></div>
    <div class="stat-line"><span>Peak gust</span><strong><span id="gustMax">--</span> m/s</strong></div>
  </div>

      <div class="card">
        <h2 class="purple">Dominant Directions</h2>
        <div id="dirCounts" class="direction-list small">No direction data yet</div>
      </div>
    </div>
  </section>

  <section class="section">
    <div class="section-title">📶 4. Connectivity & System</div>

    <div class="grid2">
      <div class="card">
        <h2 class="purple">Wi-Fi Signal</h2>
        <div class="big"><span id="wifiNow">--</span><span class="unit">%</span></div>
        <div class="meter"><div id="wifiFill" class="fill wifi"></div></div>
        <div class="mini-chart" id="chart_wifi"></div>
        <div class="stat-line"><span>Average</span><strong><span id="wifiAvg">--</span>%</strong></div>
        <div class="stat-line"><span>Best</span><strong><span id="wifiMax">--</span>%</strong></div>
        <div class="stat-line"><span>Lowest</span><strong><span id="wifiMin">--</span>%</strong></div>
      </div>

      <div class="card">
        <h2 class="cyan">System</h2>
        <div class="small">
          IP: <strong id="ipNow">--</strong><br>
          Uptime: <strong id="uptimeNow">--</strong><br>
          Heltec power: <strong id="heltecNow">--</strong>
        </div>
      </div>
    </div>
  </section>

  <section class="section">
    <div class="section-title">⭐ Key Takeaways</div>

    <div class="takeaways">
      <div class="takeaway"><div class="emoji">🌡️</div><div id="takeaway1">Loading weather summary...</div></div>
      <div class="takeaway"><div class="emoji">☀️</div><div id="takeaway2">Loading power summary...</div></div>
      <div class="takeaway"><div class="emoji">📶</div><div id="takeaway3">Loading connectivity summary...</div></div>
    </div>
  </section>

  <div class="footer">
    Live dashboard from ESP32 / SD-card weather log · <a href="/cards" style="color:var(--cyan)">Engineering View</a>
  </div>

</div>

<script>
let currentRange = '24h';

function clamp(n, min, max) {
  return Math.max(min, Math.min(max, n));
}

function setActiveButton(range) {
  document.getElementById('btn24h').classList.remove('active');
  document.getElementById('btn7d').classList.remove('active');
  document.getElementById('btn30d').classList.remove('active');

  if (range === '24h') document.getElementById('btn24h').classList.add('active');
  if (range === '7d') document.getElementById('btn7d').classList.add('active');
  if (range === '30d') document.getElementById('btn30d').classList.add('active');
}

async function setRange(range) {
  currentRange = range;
  setActiveButton(range);
  await loadHistory();
}

async function loadLive() {
  try {
    const res = await fetch('/data');
    const d = await res.json();

    document.getElementById('tempNow').textContent = d.temperature.toFixed(1);
    document.getElementById('humNow').textContent = d.humidity.toFixed(1);
    document.getElementById('pressureNow').textContent = d.pressure.toFixed(1);
    document.getElementById('luxNow').textContent = d.lux.toFixed(0);

    document.getElementById('solarNow').textContent = d.solar_power.toFixed(0);
    document.getElementById('batteryNow').textContent = d.battery_percent;
    document.getElementById('liveBattery').textContent = d.battery_percent + '%';

    document.getElementById('windNow').textContent = d.wind_mph.toFixed(1);
    document.getElementById('dirNow').textContent = d.wind_dir;

    if (typeof d.wind_degrees !== 'undefined') {
      document.getElementById('windNeedle').style.transform =
        'translate(-50%, -95%) rotate(' + d.wind_degrees + 'deg)';
    }

    document.getElementById('wifiNow').textContent = d.wifi_percent;
    document.getElementById('liveWifi').textContent = d.wifi_percent + '%';

    document.getElementById('ipNow').textContent = d.ip_address;
    document.getElementById('uptimeNow').textContent = d.uptime;
    document.getElementById('heltecNow').textContent = d.heltec_power;

    document.getElementById('batteryFill').style.width = clamp(d.battery_percent, 0, 100) + '%';
    document.getElementById('wifiFill').style.width = clamp(d.wifi_percent, 0, 100) + '%';
    document.getElementById('humFill').style.width = clamp(d.humidity, 0, 100) + '%';

    const solarWidth = clamp((d.solar_power / 1000) * 100, 0, 100);
    document.getElementById('solarFill').style.width = solarWidth + '%';
    updateConditionBanner(d);

  } catch(e) {}
}

function drawMiniChart(id, values, colourClass, unit) {
  const el = document.getElementById(id);

  if (!el || !values || values.length < 2) {
    if (el) el.innerHTML = '<div class="chart-label">waiting for data</div>';
    return;
  }

  const w = 320;
  const h = 105;
  const padX = 10;
  const padY = 12;

  let min = Math.min(...values);
  let max = Math.max(...values);

  if (min === max) {
    min -= 1;
    max += 1;
  }

  let line = '';
  let area = '';

  values.forEach((v, i) => {
    const x = padX + (i / (values.length - 1)) * (w - padX * 2);
    const y = h - padY - ((v - min) / (max - min)) * (h - padY * 2);

    line += (i === 0 ? 'M' : 'L') + x.toFixed(1) + ' ' + y.toFixed(1) + ' ';

    if (i === 0) {
      area += 'M' + x.toFixed(1) + ' ' + (h - padY).toFixed(1) + ' ';
      area += 'L' + x.toFixed(1) + ' ' + y.toFixed(1) + ' ';
    } else {
      area += 'L' + x.toFixed(1) + ' ' + y.toFixed(1) + ' ';
    }

    if (i === values.length - 1) {
      area += 'L' + x.toFixed(1) + ' ' + (h - padY).toFixed(1) + ' Z';
    }
  });

  const label = min.toFixed(1) + unit + ' → ' + max.toFixed(1) + unit;

  el.innerHTML =
    '<div class="chart-label">' + label + '</div>' +
    '<svg viewBox="0 0 ' + w + ' ' + h + '" preserveAspectRatio="none">' +
      '<path class="chart-area ' + colourClass + '" d="' + area + '"/>' +
      '<path class="chart-line ' + colourClass + '" d="' + line + '"/>' +
    '</svg>';
}

async function safeReboot() {
  const sure = confirm('Reboot the ESP32 weather station now?');

  if (!sure) {
    return;
  }

  try {
    await fetch('/reboot');

    document.body.innerHTML = `
      <div style="
        min-height:100vh;
        display:flex;
        align-items:center;
        justify-content:center;
        background:#06111f;
        color:#70e8ff;
        font-family:Arial;
        text-align:center;
        padding:30px;
      ">
        <div>
          <h1>Rebooting ESP32...</h1>
          <p>Wait a few seconds, then refresh the page.</p>
          <p><a href="/" style="color:#70e8ff;">Return to dashboard</a></p>
        </div>
      </div>
    `;
  } catch (e) {
    alert('Reboot command failed.');
  }
}

function updateConditionBanner(d) {
  let title = 'Live Weather Station';
  let emoji = '🌦️';
  let sub = 'Current outdoor conditions from your solar weather station.';

  if (d.lux < 20) {
    title = 'Night Conditions';
    emoji = '🌙';
    sub = 'Low light level detected. Station is running in night conditions.';
  } else if (d.lux > 15000) {
    title = 'Bright Sunlight';
    emoji = '☀️';
    sub = 'Strong light detected. Good time to check solar charging performance.';
  } else if (d.wind_mph > 10) {
    title = 'Breezy Conditions';
    emoji = '💨';
    sub = 'Wind speed is elevated compared with calm conditions.';
  } else if (d.humidity > 85) {
    title = 'Humid Conditions';
    emoji = '💧';
    sub = 'Humidity is high. Watch for condensation around outdoor electronics.';
  }

  document.getElementById('conditionEmoji').textContent = emoji;
  document.getElementById('conditionTitle').textContent = title;
  document.getElementById('conditionSub').textContent = sub;
  document.getElementById('conditionTemp').textContent = d.temperature.toFixed(1);

  document.getElementById('chipBattery').textContent = d.battery_percent + '%';
  document.getElementById('chipSolar').textContent = d.solar_power.toFixed(0) + ' mW';
  document.getElementById('chipWind').textContent = d.wind_mph.toFixed(1) + ' mph';
  document.getElementById('chipWifi').textContent = d.wifi_percent + '%';
}

async function loadHistory() {
  try {
    const res = await fetch('/history?range=' + currentRange);
    const h = await res.json();

    if (!h.ok) {
      document.getElementById('samples').textContent = '0';
      document.getElementById('takeaway1').textContent = h.message || 'No history data available.';
      document.getElementById('takeaway2').textContent = 'Let the logger collect data, then this panel will fill in.';
      document.getElementById('takeaway3').textContent = 'Engineering view is still available from /cards.';
      return;
    }

    document.getElementById('samples').textContent = h.rows;
    document.getElementById('rangeLabel').textContent = h.range;

    document.getElementById('tempAvg').textContent = h.temp_avg.toFixed(1);
    document.getElementById('tempMin').textContent = h.temp_min.toFixed(1);
    document.getElementById('tempMax').textContent = h.temp_max.toFixed(1);

    document.getElementById('humAvg').textContent = h.humidity_avg.toFixed(1);
    document.getElementById('humMin').textContent = h.humidity_min.toFixed(1);
    document.getElementById('humMax').textContent = h.humidity_max.toFixed(1);

    document.getElementById('pressureStart').textContent = h.pressure_start.toFixed(1);
    document.getElementById('pressureEnd').textContent = h.pressure_end.toFixed(1);
    document.getElementById('pressureTrend').textContent = h.pressure_trend;

    document.getElementById('solarPeak').textContent = h.solar_power_max.toFixed(0);
    document.getElementById('luxPeak').textContent = h.lux_max.toFixed(0);

    document.getElementById('batteryAvg').textContent = h.battery_avg.toFixed(1);
    document.getElementById('batteryMin').textContent = h.battery_min.toFixed(3);
    document.getElementById('netAvg').textContent = h.net_current_avg.toFixed(1);

    document.getElementById('windMax').textContent = h.wind_max_kph.toFixed(1);
    document.getElementById('gustMax').textContent = h.gust_max_ms.toFixed(1);

    document.getElementById('wifiAvg').textContent = h.wifi_avg.toFixed(1);
    document.getElementById('wifiMax').textContent = h.wifi_max.toFixed(1);
    document.getElementById('wifiMin').textContent = h.wifi_min.toFixed(1);

    let maxCount = 1;
    h.direction_counts.forEach(x => {
      if (x.count > maxCount) maxCount = x.count;
    });

    let dirHtml = '';
    h.direction_counts
      .sort((a,b) => b.count - a.count)
      .slice(0, 6)
      .forEach(x => {
        const w = clamp((x.count / maxCount) * 100, 3, 100);
        dirHtml += '<div class="dir-row"><strong>' + x.dir + '</strong><div class="dir-bar"><div style="width:' + w + '%"></div></div><span>' + x.count + '</span></div>';
      });

    document.getElementById('dirCounts').innerHTML = dirHtml || 'No direction data';

    drawMiniChart('chart_temp', h.chart_temp, 'orange', '°C');
    drawMiniChart('chart_humidity', h.chart_humidity, 'green', '%');
    drawMiniChart('chart_pressure', h.chart_pressure, '', ' hPa');
    drawMiniChart('chart_solar', h.chart_solar, 'yellow', ' mW');
    drawMiniChart('chart_battery', h.chart_battery, 'green', '%');
    drawMiniChart('chart_wind', h.chart_wind, '', ' km/h');
    drawMiniChart('chart_wifi', h.chart_wifi, 'purple', '%');
    
    document.getElementById('takeaway1').textContent =
      'Temperature ranged from ' + h.temp_min.toFixed(1) + '°C to ' + h.temp_max.toFixed(1) +
      '°C, with an average of ' + h.temp_avg.toFixed(1) + '°C.';

    document.getElementById('takeaway2').textContent =
      'Peak solar output was ' + h.solar_power_max.toFixed(0) +
      ' mW. Average net current was ' + h.net_current_avg.toFixed(1) + ' mA.';

    document.getElementById('takeaway3').textContent =
      'Pressure was ' + h.pressure_trend +
      '. Wi-Fi averaged ' + h.wifi_avg.toFixed(1) +
      '%, with a low of ' + h.wifi_min.toFixed(1) + '%.';

  } catch(e) {
    document.getElementById('takeaway1').textContent = 'Failed to load history data.';
  }
}

loadLive();
loadHistory();
setInterval(loadLive, 2000);
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
  json += "\"wifi_reconnects\":" + String(wifiReconnectCount) + ",";
  json += "\"fallback_ap\":\"" + String(fallbackAPActive ? "on" : "off") + "\",";

  json += "\"status_bme280\":\"" + sensorStatus(lastSeenBME280) + "\",";
  json += "\"status_bh1750\":\"" + sensorStatus(lastSeenBH1750) + "\",";
  json += "\"status_ina_load\":\"" + sensorStatus(lastSeenINALoad) + "\",";
  json += "\"status_ina_solar\":\"" + sensorStatus(lastSeenINASolar) + "\",";
  json += "\"status_wind_speed\":\"" + sensorStatus(lastSeenWindSpeed) + "\",";
  json += "\"status_wind_dir\":\"" + sensorStatus(lastSeenWindDirection) + "\",";
  json += "\"status_sd\":\"" + String(sdOK ? sensorStatus(lastSeenSD) : "offline") + "\",";

  json += "\"uptime\":\"" + latestUptime + "\",";
  json += "\"free_heap\":" + String(latestFreeHeap) + ",";
  json += "\"reset_reason\":\"" + latestResetReason + "\",";
  json += "\"wifi_mode\":\"" + latestWiFiMode + "\",";
  json += "\"ota_hostname\":\"" + latestOTAHostname + "\",";
  json += "\"low_voltage_status\":\"" + latestLowVoltageStatus + "\",";
  json += "\"low_voltage_lockout\":\"" + String(lowVoltageLockout ? "on" : "off") + "\",";
  json += "\"heltec_requested\":\"" + String(heltecRequestedOn ? "on" : "off") + "\",";
  json += "\"heltec_protection\":\"" + latestHeltecProtection + "\",";

  json += "\"heltec_power\":\"" + String(heltecPowerOn ? "on" : "off") + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleDownloadLog() {
  if (!sdOK || !SD.exists("/weather.csv")) {
    server.send(404, "text/plain", "weather.csv not found");
    return;
  }

  File file = SD.open("/weather.csv", FILE_READ);

  if (!file) {
    server.send(500, "text/plain", "Could not open weather.csv");
    return;
  }

  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=weather.csv");
  server.sendHeader("Connection", "close");

  server.streamFile(file, "text/csv");
  file.close();
}

void handleReportPage() {
  server.send_P(200, "text/html", REPORT_PAGE);
}

void handleClearLog() {
  if (!sdOK) {
    server.send(500, "application/json", "{\"ok\":false,\"message\":\"SD card not available\"}");
    return;
  }

  if (SD.exists("/weather.csv")) {
    SD.remove("/weather.csv");
  }

  File file = SD.open("/weather.csv", FILE_WRITE);

  if (!file) {
    server.send(500, "application/json", "{\"ok\":false,\"message\":\"Could not recreate weather.csv\"}");
    return;
  }

  writeCSVHeader(file);
  file.close();

  lastSeenSD = millis();

  server.send(200, "application/json", "{\"ok\":true,\"message\":\"Log cleared\"}");
}

void startFallbackAP() {
  if (fallbackAPActive) {
    return;
  }

  Serial.println("Starting fallback AP...");

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  fallbackAPActive = true;

  latestIPAddress = WiFi.softAPIP().toString();

  Serial.print("Fallback AP started: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP address: ");
  Serial.println(latestIPAddress);
}

void stopFallbackAP() {
  if (!fallbackAPActive) {
    return;
  }

  Serial.println("Stopping fallback AP...");

  WiFi.softAPdisconnect(true);
  fallbackAPActive = false;

  WiFi.mode(WIFI_STA);

  if (WiFi.status() == WL_CONNECTED) {
    latestIPAddress = WiFi.localIP().toString();
  }
}

void beginWiFiConnect() {
  Serial.println("Attempting WiFi connection...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  lastReconnectAttempt = millis();
}

void setupWiFiAndServer() {
  Serial.println();
  Serial.println("Starting WiFi...");

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiWasConnected = true;
    wifiLostAt = 0;

    latestIPAddress = WiFi.localIP().toString();

    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(latestIPAddress);

    configTime(0, 3600, "pool.ntp.org", "time.nist.gov");

    if (MDNS.begin("weatherstation")) {
      Serial.println("mDNS started: http://weatherstation.local/");
    }
  } else {
    wifiWasConnected = false;
    wifiLostAt = millis();

    Serial.println("WiFi failed during startup.");
    startFallbackAP();
  }

  server.on("/cards", handleRoot);
  server.on("/", handleReportPage);
  server.on("/history", handleHistory);
  server.on("/data", handleData);

  server.on("/heltec/on", handleHeltecOn);
  server.on("/heltec/off", handleHeltecOff);

  server.on("/download-log", handleDownloadLog);
  server.on("/clear-log", handleClearLog);

  server.on("/reboot", handleReboot);

  server.begin();
  Serial.println("Web server started.");
}

void updateWeatherData() {
  latestUptime = formatUptime(millis());
  latestFreeHeap = ESP.getFreeHeap();

  if (fallbackAPActive && WiFi.status() == WL_CONNECTED) {
    latestWiFiMode = "STA + AP";
  } else if (fallbackAPActive) {
    latestWiFiMode = "AP";
  } else if (WiFi.status() == WL_CONNECTED) {
    latestWiFiMode = "STA";
  } else {
    latestWiFiMode = "offline";
  }
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

  latestBatteryVoltage = readBatteryVoltage();
  latestBatteryPercent = batteryPercentFromVoltage(latestBatteryVoltage);
  maintainLowVoltageProtection();

  // BME280
  if (bmeOK) {
    latestTemperature = bme.readTemperature();
    latestHumidity = bme.readHumidity();
    latestPressure = bme.readPressure() / 100.0F;
    latestBmeOK = true;
    lastSeenBME280 = millis();
  } else {
    latestBmeOK = false;
  }

  // BH1750
  if (bh1750OK) {
    latestLux = lightMeter.readLightLevel();
    latestBh1750OK = true;
    lastSeenBH1750 = millis();
  } else {
    latestBh1750OK = false;
  }

  // INA219 load
  if (inaLoadOK) {
    latestLoadVoltage = inaLoad.getBusVoltage_V();
    latestLoadCurrentMA = inaLoad.getCurrent_mA();
    latestLoadPowerMW = inaLoad.getPower_mW();
    latestLoadOK = true;
    lastSeenINALoad = millis();
  } else {
    latestLoadOK = false;
  }

  // INA219 solar
  if (inaSolarOK) {
    latestSolarVoltage = inaSolar.getBusVoltage_V();
    latestSolarCurrentMA = inaSolar.getCurrent_mA();
    latestSolarPowerMW = inaSolar.getPower_mW();
    latestSolarOK = true;
    lastSeenINASolar = millis();
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
    lastSeenWindSpeed = millis();
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
  if (latestWindDirOK) {
  lastSeenWindDirection = millis();
}
}

//------Web Handlers ---------------
void handleHeltecOn() {
  heltecRequestedOn = true;

  if (lowVoltageLockout) {
    applyHeltecPower(false);
    server.send(200, "application/json", "{\"ok\":false,\"message\":\"Low voltage lockout active\",\"heltec_power\":\"off\"}");
    return;
  }

  applyHeltecPower(true);
  server.send(200, "application/json", "{\"ok\":true,\"heltec_power\":\"on\"}");
}

void handleHeltecOff() {
  heltecRequestedOn = false;
  applyHeltecPower(false);

  server.send(200, "application/json", "{\"ok\":true,\"heltec_power\":\"off\"}");
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

  return adcVoltage * BATTERY_DIVIDER_RATIO * BATTERY_CALIBRATION;
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

String sensorStatus(unsigned long lastSeen) {
  if (lastSeen == 0) {
    return "offline";
  }

  unsigned long age = millis() - lastSeen;

  if (age <= SENSOR_STALE_TIME) {
    return "ok";
  }

  return "stale";
}

unsigned long sensorAgeSeconds(unsigned long lastSeen) {
  if (lastSeen == 0) {
    return 999999;
  }

  return (millis() - lastSeen) / 1000;
}

void writeCSVHeader(File &file) {
  file.println(
    "timestamp,"
    "temperature_c,"
    "humidity_percent,"
    "pressure_hpa,"
    "lux,"
    "battery_voltage,"
    "battery_percent,"
    "load_voltage,"
    "load_current_ma,"
    "load_power_mw,"
    "solar_voltage,"
    "solar_current_ma,"
    "solar_power_mw,"
    "net_current_ma,"
    "battery_state,"
    "wind_ms,"
    "wind_kph,"
    "wind_mph,"
    "wind_avg_ms,"
    "wind_gust_ms,"
    "wind_dir,"
    "wind_degrees,"
    "wifi_rssi,"
    "wifi_percent,"
    "heltec_power"
  );
}

void setupSDCard() {
  Serial.println();
  Serial.println("Starting SD card...");

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI)) {
    sdOK = false;
    Serial.println("SD card mount FAILED.");
    return;
  }

  sdOK = true;
  lastSeenSD = millis();

  Serial.println("SD card mounted OK.");

  if (!SD.exists("/weather.csv")) {
    File file = SD.open("/weather.csv", FILE_WRITE);

    if (file) {
      writeCSVHeader(file);
      file.close();
      Serial.println("Created /weather.csv with header.");
    } else {
      Serial.println("Could not create /weather.csv.");
    }
  }
}

String getTimestamp() {
  struct tm timeinfo;

  if (getLocalTime(&timeinfo)) {
    char buffer[25];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buffer);
  }

  return "millis_" + String(millis());
}

void logWeatherToSD() {
  if (!sdOK) {
    return;
  }

  File file = SD.open("/weather.csv", FILE_APPEND);

  if (!file) {
    Serial.println("SD log failed: could not open /weather.csv");
    sdOK = false;
    return;
  }

  file.print(getTimestamp());
  file.print(",");

  file.print(latestTemperature, 2);
  file.print(",");
  file.print(latestHumidity, 2);
  file.print(",");
  file.print(latestPressure, 2);
  file.print(",");
  file.print(latestLux, 2);
  file.print(",");

  file.print(latestBatteryVoltage, 3);
  file.print(",");
  file.print(latestBatteryPercent);
  file.print(",");

  file.print(latestLoadVoltage, 3);
  file.print(",");
  file.print(latestLoadCurrentMA, 2);
  file.print(",");
  file.print(latestLoadPowerMW, 2);
  file.print(",");

  file.print(latestSolarVoltage, 3);
  file.print(",");
  file.print(latestSolarCurrentMA, 2);
  file.print(",");
  file.print(latestSolarPowerMW, 2);
  file.print(",");

  file.print(latestNetCurrentMA, 2);
  file.print(",");
  file.print(latestBatteryState);
  file.print(",");

  file.print(latestWindMS, 2);
  file.print(",");
  file.print(latestWindKPH, 2);
  file.print(",");
  file.print(latestWindMPH, 2);
  file.print(",");

  file.print(latestWindAvgMS, 2);
  file.print(",");
  file.print(latestWindGustMS, 2);
  file.print(",");

  file.print(latestWindDirName);
  file.print(",");
  file.print(latestWindDirDegrees, 1);
  file.print(",");

  file.print(latestWiFiRSSI);
  file.print(",");
  file.print(latestWiFiPercent);
  file.print(",");

  file.println(heltecPowerOn ? "on" : "off");

  file.close();

  lastSeenSD = millis();
  Serial.println("Logged weather data to SD.");
}

String formatUptime(unsigned long ms) {
  unsigned long seconds = ms / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;

  seconds %= 60;
  minutes %= 60;
  hours %= 24;

  String out = "";

  if (days > 0) {
    out += String(days) + "d ";
  }

  if (hours > 0 || days > 0) {
    out += String(hours) + "h ";
  }

  if (minutes > 0 || hours > 0 || days > 0) {
    out += String(minutes) + "m ";
  }

  out += String(seconds) + "s";

  return out;
}

String resetReasonText() {
  esp_reset_reason_t reason = esp_reset_reason();

  switch (reason) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_EXT:      return "external reset";
    case ESP_RST_SW:       return "software reset";
    case ESP_RST_PANIC:    return "panic/crash";
    case ESP_RST_INT_WDT:  return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_DEEPSLEEP:return "deep sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO:     return "SDIO reset";
    default:               return "unknown";
  }
}

void setupOTA() {
  Serial.println();
  Serial.println("Starting OTA...");

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println();
    Serial.println("OTA update started");

    // Optional safety: turn off Heltec during OTA if you want to reduce load.
    // digitalWrite(HELTEC_POWER_PIN, LOW);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println();
    Serial.println("OTA update finished");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA progress: %u%%\r", (progress * 100) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error[%u]: ", error);

    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End failed");
    } else {
      Serial.println("Unknown error");
    }
  });

  ArduinoOTA.begin();
  latestOTAHostname = String(OTA_HOSTNAME);
  Serial.print("OTA ready. Hostname: ");
  Serial.println(OTA_HOSTNAME);
}

void maintainLowVoltageProtection() {
  if (latestBatteryVoltage <= BATTERY_CRITICAL_VOLTAGE) {
    latestLowVoltageStatus = "critical";
  } else if (latestBatteryVoltage <= BATTERY_HELTEC_OFF_VOLTAGE) {
    latestLowVoltageStatus = "heltec off";
  } else if (latestBatteryVoltage <= BATTERY_WARN_VOLTAGE) {
    latestLowVoltageStatus = "warning";
  } else {
    latestLowVoltageStatus = "normal";
  }

  // Enter low-voltage lockout
  if (!lowVoltageLockout && latestBatteryVoltage <= BATTERY_HELTEC_OFF_VOLTAGE) {
    lowVoltageLockout = true;
    latestHeltecProtection = "locked out";

    applyHeltecPower(false);

    Serial.println("LOW VOLTAGE: Heltec forced OFF.");
  }

  // Recover from low-voltage lockout
  if (lowVoltageLockout && latestBatteryVoltage >= BATTERY_RECOVER_VOLTAGE) {
    lowVoltageLockout = false;
    latestHeltecProtection = "recovered";

    Serial.println("Battery recovered: Heltec power allowed again.");

    // If user had previously requested it on, restore it automatically.
    if (heltecRequestedOn) {
      applyHeltecPower(true);
      Serial.println("Heltec restored ON after battery recovery.");
    }
  }

  if (!lowVoltageLockout) {
    latestHeltecProtection = "allowed";
  }
}

// ----------------------------------------------------
// Setup
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  latestResetReason = resetReasonText();
  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32 Weather Station - Basic Sketch");
  Serial.println("====================================");

  pinMode(HELTEC_POWER_PIN, OUTPUT);

  heltecRequestedOn = false;
  applyHeltecPower(false);

  Wire.begin(I2C_SDA, I2C_SCL);

  scanI2C();

  setupBME280();
  setupBH1750();
  setupINA219();
  setupWindSensors();

  pinMode(BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);

  Serial.println();
  setupSDCard();
  setupWiFiAndServer();
  setupOTA();
  updateWeatherData();
  Serial.println("Setup complete.");
  lastGustReset = millis();

}

void maintainWiFi() {
  if (millis() - lastWiFiCheck < WIFI_CHECK_INTERVAL) {
    return;
  }

  lastWiFiCheck = millis();

  wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    latestIPAddress = WiFi.localIP().toString();

    if (!wifiWasConnected) {
      Serial.println("WiFi reconnected.");
      Serial.print("IP address: ");
      Serial.println(latestIPAddress);

      wifiReconnectCount++;

      configTime(0, 3600, "pool.ntp.org", "time.nist.gov");
    }

    wifiWasConnected = true;
    wifiLostAt = 0;

    // Optional: once router WiFi is back, turn off fallback AP.
    if (fallbackAPActive) {
      stopFallbackAP();
    }

    return;
  }

  // WiFi is not connected
  if (wifiWasConnected) {
    Serial.println("WiFi connection lost.");
    wifiLostAt = millis();
  }

  wifiWasConnected = false;

  if (wifiLostAt == 0) {
    wifiLostAt = millis();
  }

  // Start fallback AP after being offline for a while
  if (!fallbackAPActive && millis() - wifiLostAt >= WIFI_AP_FALLBACK_AFTER) {
    startFallbackAP();
  }

  // Try reconnecting every so often
  if (millis() - lastReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
    Serial.println("Retrying WiFi connection...");

    WiFi.mode(fallbackAPActive ? WIFI_AP_STA : WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    lastReconnectAttempt = millis();
  }

  if (fallbackAPActive) {
    latestIPAddress = WiFi.softAPIP().toString();
  }
}

// ----------------------------------------------------
// Main loop
// ----------------------------------------------------
void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  maintainWiFi();

  if (rebootRequested && millis() - rebootRequestedAt >= REBOOT_DELAY_MS) {
  Serial.println("Rebooting now...");
  delay(100);
  ESP.restart();
}

  if (millis() - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdate = millis();

    updateWeatherData();

    if (millis() - lastSDLog >= SD_LOG_INTERVAL) {
      lastSDLog = millis();
      logWeatherToSD();
    }

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

    Serial.print("Status:      ");
    Serial.print("BME280=");
    Serial.print(sensorStatus(lastSeenBME280));
    Serial.print(" BH1750=");
    Serial.print(sensorStatus(lastSeenBH1750));
    Serial.print(" LoadINA=");
    Serial.print(sensorStatus(lastSeenINALoad));
    Serial.print(" SolarINA=");
    Serial.print(sensorStatus(lastSeenINASolar));
    Serial.print(" WindSpd=");
    Serial.print(sensorStatus(lastSeenWindSpeed));
    Serial.print(" WindDir=");
    Serial.print(sensorStatus(lastSeenWindDirection));
    Serial.print(" SD=");
    Serial.println(sdOK ? sensorStatus(lastSeenSD) : "offline");

    Serial.print("Uptime:      ");
    Serial.print(latestUptime);
    Serial.print(" | Heap: ");
    Serial.print(latestFreeHeap);
    Serial.print(" | Reset: ");
    Serial.println(latestResetReason);

    Serial.print("LV Protect:  ");
    Serial.print(latestLowVoltageStatus);
    Serial.print(" | Lockout: ");
    Serial.print(lowVoltageLockout ? "ON" : "OFF");
    Serial.print(" | Heltec: ");
    Serial.print(heltecPowerOn ? "ON" : "OFF");
    Serial.print(" | Requested: ");
    Serial.println(heltecRequestedOn ? "ON" : "OFF");

    Serial.println("==================================");
  }
}