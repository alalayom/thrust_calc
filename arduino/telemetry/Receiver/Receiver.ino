#include <SPI.h>
#include <LoRa.h>

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 2

#define LED_PIN 7

/*
  EXPECTED DATA PACKET FORMAT:
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

String getCsvValue(String pData, int pIndex) {
  int tStartIndex = 0;
  int tEndIndex = -1;
  int tCurrentIndex = 0;

  while (true) {
    tStartIndex = tEndIndex + 1;
    tEndIndex = pData.indexOf(',', tStartIndex);

    if (tEndIndex == -1) {
      tEndIndex = pData.length();
    }

    if (tCurrentIndex == pIndex) {
      return pData.substring(tStartIndex, tEndIndex);
    }

    if (tEndIndex >= pData.length()) {
      break;
    }

    tCurrentIndex++;
  }

  return "";
}

void blinkLed() {
  digitalWrite(LED_PIN, HIGH);
  delay(40);
  digitalWrite(LED_PIN, LOW);
}

void printTelemetry(String pMessage) {
  String tAx = getCsvValue(pMessage, 1);
  String tAy = getCsvValue(pMessage, 2);
  String tAz = getCsvValue(pMessage, 3);
  String tGx = getCsvValue(pMessage, 4);
  String tGy = getCsvValue(pMessage, 5);
  String tGz = getCsvValue(pMessage, 6);
  String tBmpTemp = getCsvValue(pMessage, 7);
  String tPressure = getCsvValue(pMessage, 8);
  String tAltitude = getCsvValue(pMessage, 9);

  Serial.println("Telemetry data received:");
  Serial.print("  Accelerometer X: ");
  Serial.println(tAx);
  Serial.print("  Accelerometer Y: ");
  Serial.println(tAy);
  Serial.print("  Accelerometer Z: ");
  Serial.println(tAz);

  Serial.print("  Gyroscope X: ");
  Serial.println(tGx);
  Serial.print("  Gyroscope Y: ");
  Serial.println(tGy);
  Serial.print("  Gyroscope Z: ");
  Serial.println(tGz);

  Serial.print("  BMP280 Temperature C: ");
  Serial.println(tBmpTemp);
  Serial.print("  Pressure hPa: ");
  Serial.println(tPressure);
  Serial.print("  Estimated Altitude m: ");
  Serial.println(tAltitude);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Starting receiver..");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("LoRa receiver starting...");

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed.");
    while (1);
  }

  LoRa.setSyncWord(0x12);

  Serial.println("LoRa initialized successfully.");
}

void loop() {
  int tPacketSize = LoRa.parsePacket();

  if (tPacketSize > 0) {
    String tMessage = "";

    while (LoRa.available()) {
      tMessage += (char)LoRa.read();
    }

    Serial.println("----------------");
    Serial.print("Raw packet: ");
    Serial.println(tMessage);

    Serial.print("Packet size: ");
    Serial.println(tPacketSize);

    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());

    Serial.print("SNR: ");
    Serial.println(LoRa.packetSnr());

    if (tMessage == "S") {  //start
      Serial.println("S");
    }
    else if (tMessage == "T") { //stop
      digitalWrite(LED_PIN, LOW);
      Serial.println("T"); 
    }
    else if (tMessage.startsWith("D,")) {
      blinkLed();
      //printTelemetry(tMessage); -> for Serial Monitor
      Serial.println(tMessage.substring(2));
    }
    else {
      Serial.println("Unknown packet received.");
    }
  }
}