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

  Serial.println();
  Serial.println("Setup complete.");
  lastGustReset = millis();
}

// ----------------------------------------------------
// Main loop
// ----------------------------------------------------
void loop() {
  Serial.println();
  Serial.println("========== WEATHER DATA ==========");

  // --------------------------------------------------
  // BME280
  // --------------------------------------------------
  if (bmeOK) {
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressureHPa = bme.readPressure() / 100.0F;

    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.println(" C");

    Serial.print("Humidity:    ");
    Serial.print(humidity, 1);
    Serial.println(" %");

    Serial.print("Pressure:    ");
    Serial.print(pressureHPa, 1);
    Serial.println(" hPa");
  } else {
    Serial.println("BME280:      not available");
  }

  // --------------------------------------------------
  // BH1750
  // --------------------------------------------------
  if (bh1750OK) {
    float lux = lightMeter.readLightLevel();

    Serial.print("Light:       ");
    Serial.print(lux, 1);
    Serial.println(" lux");
  } else {
    Serial.println("BH1750:      not available");
  }

float loadCurrentMA = 0;
float solarCurrentMA = 0;

bool loadCurrentValid = false;
bool solarCurrentValid = false;

// --------------------------------------------------
// INA219 #1
// --------------------------------------------------
if (inaLoadOK) {
  float loadVoltage = inaLoad.getBusVoltage_V();
  loadCurrentMA = inaLoad.getCurrent_mA();
  loadCurrentValid = true;
  float loadPowerMW = inaLoad.getPower_mW();

  Serial.println("Station Load:");
  Serial.print("  Voltage: ");
  Serial.print(loadVoltage, 3);
  Serial.println(" V");

  Serial.print("  Current: ");
  Serial.print(loadCurrentMA, 2);
  Serial.println(" mA");

  Serial.print("  Power:   ");
  Serial.print(loadPowerMW, 2);
  Serial.println(" mW");
} else {
  Serial.println("Station Load INA219: not available");
}

if (inaSolarOK) {
  float solarVoltage = inaSolar.getBusVoltage_V();
  solarCurrentMA = inaSolar.getCurrent_mA();
  solarCurrentValid = true;
  float solarPowerMW = inaSolar.getPower_mW();

  Serial.println("Solar Charge:");
  Serial.print("  Voltage: ");
  Serial.print(solarVoltage, 3);
  Serial.println(" V");

  Serial.print("  Current: ");
  Serial.print(solarCurrentMA, 2);
  Serial.println(" mA");

  Serial.print("  Power:   ");
  Serial.print(solarPowerMW, 2);
  Serial.println(" mW");
} else {
  Serial.println("Solar Charge INA219: not available");
}

// --------------------------------------------------
// WIND SPEED + AVERAGE + GUST
// --------------------------------------------------
float windMS = 0;
float windKPH = 0;
float windMPH = 0;
uint16_t windSpeedRaw = 0;

bool speedOK = readWindSpeed(windMS, windKPH, windMPH, windSpeedRaw);

if (speedOK) {
  addWindSample(windMS);
  resetGustIfNeeded();

  float avgWindMS = getAverageWindMS();
  float avgWindKPH = avgWindMS * 3.6;
  float avgWindMPH = avgWindMS * 2.23694;

  Serial.print("Wind speed:  ");
  Serial.print(windMS, 1);
  Serial.print(" m/s | ");
  Serial.print(windKPH, 1);
  Serial.print(" km/h | ");
  Serial.print(windMPH, 1);
  Serial.print(" mph | raw ");
  Serial.println(windSpeedRaw);

  Serial.print("Wind avg:    ");
  Serial.print(avgWindMS, 1);
  Serial.print(" m/s | ");
  Serial.print(avgWindKPH, 1);
  Serial.print(" km/h | ");
  Serial.print(avgWindMPH, 1);
  Serial.println(" mph");

  Serial.print("Wind gust:   ");
  Serial.print(windGustMS, 1);
  Serial.print(" m/s | ");
  Serial.print(windGustKPH, 1);
  Serial.print(" km/h | ");
  Serial.print(windGustMPH, 1);
  Serial.println(" mph");
} else {
  Serial.println("Wind speed:  read failed");
}

  // --------------------------------------------------
  // WIND DIRECTION
  // --------------------------------------------------
  uint16_t windDirRaw = 0;
  String windDirName = "ERR";
  float windDirDegrees = -1;

  bool directionOK = readWindDirection(windDirRaw, windDirName, windDirDegrees);

  if (directionOK) {
    Serial.print("Direction:   ");
    Serial.print(windDirName);
    Serial.print(" | ");
    Serial.print(windDirDegrees, 1);
    Serial.print(" degrees | raw ");
    Serial.println(windDirRaw);
  } else {
    Serial.println("Direction:   read failed");
  }
// ----------------------------------------------------
// Show Battery Balance
// ----------------------------------------------------
if (loadCurrentValid && solarCurrentValid) {
  float netCurrentMA = solarCurrentMA - loadCurrentMA;

  Serial.print("Net battery current: ");
  Serial.print(netCurrentMA, 2);
  Serial.println(" mA");

  if (netCurrentMA > 10) {
    Serial.println("Battery state: charging");
  } else if (netCurrentMA < -10) {
    Serial.println("Battery state: discharging");
  } else {
    Serial.println("Battery state: balanced");
  }
} else {
  Serial.println("Net battery current: unavailable");
}

  Serial.println("==================================");

  delay(2000);
}