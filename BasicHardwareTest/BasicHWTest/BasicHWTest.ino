/*
  ESP32 Weather Station - First Hardware Test Sketch

  Checks:
  - I2C bus on GPIO21 / GPIO22
  - BME280 / BH1750 / INA219 presence
  - SD card basic write test
  - RS485 UART ports started
  - Heltec power-control pin available

  Open Serial Monitor at 115200 baud.

  Commands:
    ? = help
    i = scan I2C
    s = test SD card
    h = toggle Heltec power pin
    r = print RS485 status
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// ---------- I2C ----------
#define I2C_SDA 21
#define I2C_SCL 22

// ---------- SD CARD ----------
#define SD_CS   5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18

// ---------- HELTEC POWER CONTROL ----------
#define HELTEC_POWER_PIN 25

// Change this later if your Heltec power circuit is active LOW
bool heltecPowerState = false;

// ---------- RS485 #1 - WIND SPEED ----------
#define RS485_SPEED_RX    16
#define RS485_SPEED_TX    17
#define RS485_SPEED_DE_RE 4

// ---------- RS485 #2 - WIND DIRECTION ----------
#define RS485_DIR_RX      33
#define RS485_DIR_TX      32
#define RS485_DIR_DE_RE   27

HardwareSerial WindSpeedSerial(1);
HardwareSerial WindDirSerial(2);

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  ? = help");
  Serial.println("  i = scan I2C bus");
  Serial.println("  s = test SD card");
  Serial.println("  h = toggle Heltec power pin");
  Serial.println("  r = RS485 status");
  Serial.println();
}

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
      if (address == 0x41) Serial.print("  possible second INA219");

      Serial.println();
      count++;
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found.");
    Serial.println("Check SDA/SCL, 3.3V, GND, and sensor power.");
  } else {
    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" I2C device(s).");
  }
}

void testSDCard() {
  Serial.println();
  Serial.println("Testing SD card...");

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD card mount FAILED.");
    Serial.println("Check CS, SCK, MOSI, MISO, 3.3V/5V module power, and GND.");
    return;
  }

  Serial.println("SD card mounted OK.");

  File testFile = SD.open("/test.txt", FILE_APPEND);

  if (!testFile) {
    Serial.println("Could not open /test.txt for writing.");
    return;
  }

  testFile.println("ESP32 weather station SD test OK");
  testFile.close();

  Serial.println("Wrote to /test.txt successfully.");

  File readFile = SD.open("/test.txt");

  if (readFile) {
    Serial.println("Reading /test.txt:");
    while (readFile.available()) {
      Serial.write(readFile.read());
    }
    readFile.close();
    Serial.println();
  } else {
    Serial.println("Could not reopen /test.txt for reading.");
  }
}

void toggleHeltecPower() {
  heltecPowerState = !heltecPowerState;
  digitalWrite(HELTEC_POWER_PIN, heltecPowerState ? HIGH : LOW);

  Serial.print("Heltec power-control pin GPIO25 set to: ");
  Serial.println(heltecPowerState ? "HIGH" : "LOW");
}

void printRS485Status() {
  Serial.println();
  Serial.println("RS485 ports configured:");

  Serial.println("Wind speed RS485:");
  Serial.println("  RX = GPIO16");
  Serial.println("  TX = GPIO17");
  Serial.println("  DE/RE = GPIO4");
  Serial.println("  Mode = receive");

  Serial.println("Wind direction RS485:");
  Serial.println("  RX = GPIO33");
  Serial.println("  TX = GPIO32");
  Serial.println("  DE/RE = GPIO27");
  Serial.println("  Mode = receive");

  Serial.println();
  Serial.println("Note: Modbus RS485 sensors usually stay silent until polled.");
  Serial.println("So no incoming data here does not mean the sensors are faulty.");
}

void setupRS485() {
  pinMode(RS485_SPEED_DE_RE, OUTPUT);
  pinMode(RS485_DIR_DE_RE, OUTPUT);

  // LOW = receive mode on most MAX485 modules when DE and RE are joined.
  digitalWrite(RS485_SPEED_DE_RE, LOW);
  digitalWrite(RS485_DIR_DE_RE, LOW);

  WindSpeedSerial.begin(9600, SERIAL_8N1, RS485_SPEED_RX, RS485_SPEED_TX);
  WindDirSerial.begin(9600, SERIAL_8N1, RS485_DIR_RX, RS485_DIR_TX);
}

void checkForRS485Bytes() {
  while (WindSpeedSerial.available()) {
    uint8_t b = WindSpeedSerial.read();
    Serial.print("[Speed RS485] 0x");
    if (b < 16) Serial.print("0");
    Serial.println(b, HEX);
  }

  while (WindDirSerial.available()) {
    uint8_t b = WindDirSerial.read();
    Serial.print("[Direction RS485] 0x");
    if (b < 16) Serial.print("0");
    Serial.println(b, HEX);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32 Weather Station Hardware Test");
  Serial.println("====================================");

  // Keep Heltec power pin defined but controlled manually.
  pinMode(HELTEC_POWER_PIN, OUTPUT);
  digitalWrite(HELTEC_POWER_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  setupRS485();

  Serial.println("I2C started on GPIO21 SDA / GPIO22 SCL.");
  Serial.println("RS485 UARTs started.");
  Serial.println("Heltec power-control pin set LOW.");

  printHelp();
  scanI2C();
}

void loop() {
  checkForRS485Bytes();

  if (Serial.available()) {
    char command = Serial.read();

    if (command == '?') {
      printHelp();
    }

    if (command == 'i' || command == 'I') {
      scanI2C();
    }

    if (command == 's' || command == 'S') {
      testSDCard();
    }

    if (command == 'h' || command == 'H') {
      toggleHeltecPower();
    }

    if (command == 'r' || command == 'R') {
      printRS485Status();
    }
  }
}