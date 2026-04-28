#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 2

#define BUTTON_PIN 3

#define MPU_ADDR 0x68
#define BMP_ADDR 0x76

Adafruit_BMP280 bmp;

bool gStreamState = false;
bool gLastButtonReading = HIGH;
bool gButtonStableState = HIGH;
bool gBmpOk = false;

unsigned long gLastDebounceTime = 0;
const unsigned long gDebounceDelay = 50;

unsigned long gLastSendTime = 0;
const unsigned long gSendInterval = 500;

int16_t gAx, gAy, gAz;
int16_t gGx, gGy, gGz;

/*
  DATA PACKET FORMAT:
  D,AX,AY,AZ,GX,GY,GZ,BT,P,ALT

  D   = Data packet marker
  AX  = Accelerometer X raw value
  AY  = Accelerometer Y raw value
  AZ  = Accelerometer Z raw value
  GX  = Gyroscope X raw value
  GY  = Gyroscope Y raw value
  GZ  = Gyroscope Z raw value
  BT  = BMP280 temperature in Celsius
  P   = Pressure in hPa
  ALT = Estimated altitude in meters
*/

void sendLoRaMessage(String pMessage) {
  LoRa.beginPacket();
  LoRa.print(pMessage);
  LoRa.endPacket();

  Serial.print("LoRa packet sent: ");
  Serial.println(pMessage);
}

bool readMPU6050() {
  if (Wire.getWireTimeoutFlag()) {
    Serial.println("I2C timeout flag detected. Clearing flag.");
    Wire.clearWireTimeoutFlag();
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  byte tError = Wire.endTransmission(false);

  if (tError != 0) {
    Serial.print("MPU6050 I2C transmission failed. Error code: ");
    Serial.println(tError);

    Wire.end();
    delay(20);
    Wire.begin();
    Wire.setWireTimeout(3000, true);

    Serial.println("I2C bus restarted after MPU6050 error.");
    return false;
  }

  byte tBytesRead = Wire.requestFrom(MPU_ADDR, 14, true);

  if (tBytesRead != 14) {
    Serial.print("MPU6050 read failed. Bytes read: ");
    Serial.println(tBytesRead);

    Wire.end();
    delay(20);
    Wire.begin();
    Wire.setWireTimeout(3000, true);

    Serial.println("I2C bus restarted after incomplete MPU6050 read.");
    return false;
  }

  gAx = ((int16_t)Wire.read() << 8) | Wire.read();
  gAy = ((int16_t)Wire.read() << 8) | Wire.read();
  gAz = ((int16_t)Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  gGx = ((int16_t)Wire.read() << 8) | Wire.read();
  gGy = ((int16_t)Wire.read() << 8) | Wire.read();
  gGz = ((int16_t)Wire.read() << 8) | Wire.read();

  return true;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Starting transmitter..");
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();
  Wire.setWireTimeout(3000, true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);

  byte tMpuError = Wire.endTransmission(true);

  if (tMpuError == 0) {
    Serial.println("MPU6050 initialized successfully.");
  } else {
    Serial.print("MPU6050 initialization failed. I2C error code: ");
    Serial.println(tMpuError);
  }

  if (bmp.begin(BMP_ADDR)) {
    gBmpOk = true;
    Serial.println("BMP280 initialized successfully.");
  } else {
    gBmpOk = false;
    Serial.println("BMP280 initialization failed.");
  }

  Serial.println("LoRa transmitter starting...");

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed.");
    while (1);
  }

  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(10);

  Serial.println("LoRa initialized successfully.");
}

void loop() {
  bool tReading = digitalRead(BUTTON_PIN);

  if (tReading != gLastButtonReading) {
    gLastDebounceTime = millis();
  }

  if ((millis() - gLastDebounceTime) > gDebounceDelay) {
    if (tReading != gButtonStableState) {
      gButtonStableState = tReading;

      if (gButtonStableState == LOW) {
        gStreamState = !gStreamState;

        if (gStreamState == true) {
          sendLoRaMessage("S");
          Serial.println("Telemetry stream started.");
        } else {
          sendLoRaMessage("T");
          Serial.println("Telemetry stream stopped.");
        }

        gLastSendTime = millis();
      }
    }
  }

  gLastButtonReading = tReading;

  if (gStreamState == false) {
    return;
  }

  if (millis() - gLastSendTime >= gSendInterval) {
    gLastSendTime = millis();

    if (readMPU6050() == false) {
      Serial.println("Telemetry packet skipped because MPU6050 read failed.");
      return;
    }

    float tBmpTemp = -999.0;
    float tPressure = -999.0;
    float tAltitude = -999.0;

    if (gBmpOk == true) {
      tBmpTemp = bmp.readTemperature();
      tPressure = bmp.readPressure() / 100.0;
      tAltitude = bmp.readAltitude(1013.25);
    }

    String tMessage = "D,";
    tMessage += String(gAx) + ",";
    tMessage += String(gAy) + ",";
    tMessage += String(gAz) + ",";
    tMessage += String(gGx) + ",";
    tMessage += String(gGy) + ",";
    tMessage += String(gGz) + ",";
    tMessage += String(tBmpTemp, 2) + ",";
    tMessage += String(tPressure, 2) + ",";
    tMessage += String(tAltitude, 2);

    sendLoRaMessage(tMessage);
  }
}