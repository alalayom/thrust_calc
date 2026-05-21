#include <LoRa.h>
#include <Adafruit_BMP3XX.h>

#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_SS    10
#define LORA_RST   9
#define LORA_DIO0  14

#define BUTTON_PIN 3

#define I2C_SDA 4
#define I2C_SCL 5

#define MPU_ADDR 0x68
#define BMP_ADDR 0x76

Adafruit_BMP3XX bmp;

bool gStreamState = false;
bool gLastButtonReading = HIGH;
bool gButtonStableState = HIGH;
bool gBmpOk = false;
bool gMpuOk = false;

unsigned long gLastDebounceTime = 0;
const unsigned long gDebounceDelay = 50;

unsigned long gLastSendTime = 0;
const unsigned long gSendInterval = 100;
const byte kBmpWarmupReadCount = 3;
byte gBmpWarmupReadingsRemaining = kBmpWarmupReadCount;

int16_t gAx, gAy, gAz;
int16_t gGx, gGy, gGz;

/*
  DATA PACKET FORMAT:
  D,AX,AY,AZ,GX,GY,GZ,BT,P,ALT
*/

void writeI2cRegister(byte pAddress, byte pRegister, byte pValue) {
  Wire.beginTransmission(pAddress);
  Wire.write(pRegister);
  Wire.write(pValue);
  Wire.endTransmission(true);
}

byte readI2cRegister(byte pAddress, byte pRegister) {
  Wire.beginTransmission(pAddress);
  Wire.write(pRegister);
  Wire.endTransmission(false);
  Wire.requestFrom(pAddress, (byte)1, true);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

void sendLoRaMessage(String pMessage) {
  LoRa.beginPacket();
  LoRa.print(pMessage);
  LoRa.endPacket();

  if (pMessage == "S" || pMessage == "T") {
    Serial.print("LoRa packet sent: ");
    Serial.println(pMessage);
  }
}

void restartI2cBus() {
  Wire.end();
  delay(20);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C bus restarted.");
}

bool initMPU6500() {
  writeI2cRegister(MPU_ADDR, 0x6B, 0x00);
  delay(100);

  byte tWhoAmI = readI2cRegister(MPU_ADDR, 0x75);

  Serial.print("MPU WHO_AM_I: 0x");
  Serial.println(tWhoAmI, HEX);

  if (tWhoAmI != 0x70) {
    return false;
  }

  writeI2cRegister(MPU_ADDR, 0x6B, 0x01); // Clock source: PLL
  delay(10);

  writeI2cRegister(MPU_ADDR, 0x1A, 0x03); // DLPF
  writeI2cRegister(MPU_ADDR, 0x1B, 0x08); // Gyro +-500 dps
  writeI2cRegister(MPU_ADDR, 0x1C, 0x08); // Accel +-4g

  return true;
}

bool readMPU6500() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  byte tError = Wire.endTransmission(false);

  if (tError != 0) {
    Serial.print("MPU6500 I2C transmission failed. Error code: ");
    Serial.println(tError);

    restartI2cBus();
    return false;
  }

  byte tBytesRead = Wire.requestFrom(MPU_ADDR, 14, true);

  if (tBytesRead != 14) {
    Serial.print("MPU6500 read failed. Bytes read: ");
    Serial.println(tBytesRead);

    restartI2cBus();
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

bool initBMP388() {
  if (!bmp.begin_I2C(BMP_ADDR, &Wire)) {
    return false;
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting ESP32-S3 transmitter..");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (initMPU6500()) {
    gMpuOk = true;
    Serial.println("MPU6500 initialized successfully.");
  } else {
    gMpuOk = false;
    Serial.println("MPU6500 initialization failed.");
  }

  if (initBMP388()) {
    gBmpOk = true;
    Serial.println("BMP388 initialized successfully.");
  } else {
    gBmpOk = false;
    Serial.println("BMP388 initialization failed.");
  }

  Serial.println("LoRa transmitter starting...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed.");
    while (1) {
      delay(1000);
    }
  }

  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(10);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

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
          for (int i = 0; i < 3; i++) {
            sendLoRaMessage("T");
            delay(50);
          }
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

    if (gMpuOk == false) {
      Serial.println("Telemetry packet skipped because MPU6500 is not initialized.");
      return;
    }

    if (readMPU6500() == false) {
      Serial.println("Telemetry packet skipped because MPU6500 read failed.");
      return;
    }

    float tBmpTemp = -999.0;
    float tPressure = -999.0;
    float tAltitude = -999.0;

    if (gBmpOk == true) {
      if (bmp.performReading()) {
        tBmpTemp = bmp.temperature;
        tPressure = bmp.pressure / 100.0;
        tAltitude = bmp.readAltitude(1013.25);

        if (gBmpWarmupReadingsRemaining > 0) {
          gBmpWarmupReadingsRemaining--;
          Serial.print("BMP388 warmup reading skipped. Pressure hPa: ");
          Serial.println(tPressure, 2);
          return;
        }
      } else {
        Serial.println("BMP388 reading failed.");
      }
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

    Serial.print("Sent telemetry: ");
    Serial.println(tMessage);
  }
}
