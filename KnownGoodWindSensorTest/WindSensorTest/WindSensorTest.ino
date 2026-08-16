/*
  ESP32 Weather Station - Dual RS485 Wind Sensor Test

  Reads:
  - RS485 wind speed sensor on UART1
  - RS485 wind direction sensor on UART2

  Hardware:
  ESP32D + 2x MAX485 modules

  Wind Speed MAX485:
    ESP32 GPIO17 TX  -> DI
    ESP32 GPIO16 RX  <- RO through voltage divider
    ESP32 GPIO4      -> DE + RE joined

  Wind Direction MAX485:
    ESP32 GPIO32 TX  -> DI
    ESP32 GPIO33 RX  <- RO through voltage divider
    ESP32 GPIO27     -> DE + RE joined

  MAX485:
    VCC -> 5V
    GND -> GND
    A/B -> RS485 sensor A/B

  Wind sensors:
    Powered separately from 10-30V supply, e.g. 12V boost converter.
    Sensor GND must be common with ESP32 GND.

  Serial Monitor:
    115200 baud
*/

#include <ModbusMaster.h>

// ----------------------------------------------------
// WIND SPEED RS485
// ----------------------------------------------------
#define WIND_SPEED_RX     16
#define WIND_SPEED_TX     17
#define WIND_SPEED_DE_RE  4

// ----------------------------------------------------
// WIND DIRECTION RS485
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

HardwareSerial WindSpeedSerial(1);
HardwareSerial WindDirSerial(2);

ModbusMaster windSpeed;
ModbusMaster windDirection;

// ----------------------------------------------------
// MAX485 direction control - wind speed
// ----------------------------------------------------
void preTransmissionSpeed() {
  digitalWrite(WIND_SPEED_DE_RE, HIGH);   // transmit mode
  delayMicroseconds(300);
}

void postTransmissionSpeed() {
  delayMicroseconds(300);
  digitalWrite(WIND_SPEED_DE_RE, LOW);    // receive mode
}

// ----------------------------------------------------
// MAX485 direction control - wind direction
// ----------------------------------------------------
void preTransmissionDirection() {
  digitalWrite(WIND_DIR_DE_RE, HIGH);     // transmit mode
  delayMicroseconds(300);
}

void postTransmissionDirection() {
  delayMicroseconds(300);
  digitalWrite(WIND_DIR_DE_RE, LOW);      // receive mode
}

// ----------------------------------------------------
// Convert 16-point compass raw value to text
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
// Setup
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("===================================");
  Serial.println("ESP32 Dual RS485 Wind Sensor Test");
  Serial.println("===================================");

  // MAX485 direction pins
  pinMode(WIND_SPEED_DE_RE, OUTPUT);
  pinMode(WIND_DIR_DE_RE, OUTPUT);

  // Start in receive mode
  digitalWrite(WIND_SPEED_DE_RE, LOW);
  digitalWrite(WIND_DIR_DE_RE, LOW);

  // Start UARTs
  WindSpeedSerial.begin(MODBUS_BAUD, SERIAL_8N1, WIND_SPEED_RX, WIND_SPEED_TX);
  WindDirSerial.begin(MODBUS_BAUD, SERIAL_8N1, WIND_DIR_RX, WIND_DIR_TX);

  // Start Modbus devices
  windSpeed.begin(MODBUS_ADDRESS, WindSpeedSerial);
  windSpeed.preTransmission(preTransmissionSpeed);
  windSpeed.postTransmission(postTransmissionSpeed);

  windDirection.begin(MODBUS_ADDRESS, WindDirSerial);
  windDirection.preTransmission(preTransmissionDirection);
  windDirection.postTransmission(postTransmissionDirection);

  Serial.println("Wind speed RS485:");
  Serial.println("  RX GPIO16, TX GPIO17, DE/RE GPIO4");

  Serial.println("Wind direction RS485:");
  Serial.println("  RX GPIO33, TX GPIO32, DE/RE GPIO27");

  Serial.println();
  Serial.println("Starting readings...");
}

// ----------------------------------------------------
// Read wind speed
// ----------------------------------------------------
bool readWindSpeed(float &ms, float &kph, float &mph, uint16_t &raw) {
  uint8_t result = windSpeed.readHoldingRegisters(WIND_REGISTER, 1);

  if (result == windSpeed.ku8MBSuccess) {
    raw = windSpeed.getResponseBuffer(0);

    // Your tested sensor uses raw / 10 = m/s
    ms = raw / 10.0;
    kph = ms * 3.6;
    mph = ms * 2.23694;

    return true;
  }

  Serial.print("Wind speed read failed. Modbus error: 0x");
  Serial.println(result, HEX);
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

  Serial.print("Wind direction read failed. Modbus error: 0x");
  Serial.println(result, HEX);
  return false;
}

// ----------------------------------------------------
// Main loop
// ----------------------------------------------------
void loop() {
  float windMS = 0;
  float windKPH = 0;
  float windMPH = 0;
  uint16_t windSpeedRaw = 0;

  uint16_t windDirRaw = 0;
  String windDirName = "ERR";
  float windDirDegrees = -1;

  bool speedOK = readWindSpeed(windMS, windKPH, windMPH, windSpeedRaw);

  // Small delay between Modbus reads
  delay(150);

  bool directionOK = readWindDirection(windDirRaw, windDirName, windDirDegrees);

  Serial.println();
  Serial.println("---------- WIND DATA ----------");

  if (speedOK) {
    Serial.print("Speed raw: ");
    Serial.print(windSpeedRaw);

    Serial.print(" | ");
    Serial.print(windMS, 1);
    Serial.print(" m/s");

    Serial.print(" | ");
    Serial.print(windKPH, 1);
    Serial.print(" km/h");

    Serial.print(" | ");
    Serial.print(windMPH, 1);
    Serial.println(" mph");
  } else {
    Serial.println("Speed: read failed");
  }

  if (directionOK) {
    Serial.print("Direction raw: ");
    Serial.print(windDirRaw);

    Serial.print(" | ");
    Serial.print(windDirName);

    Serial.print(" | ");
    Serial.print(windDirDegrees, 1);
    Serial.println(" degrees");
  } else {
    Serial.println("Direction: read failed");
  }

  Serial.println("-------------------------------");

  delay(1000);
}