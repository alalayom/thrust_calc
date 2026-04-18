#include <HX711_ADC.h>

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

const int kHx711DoutPin = 4;
const int kHx711SckPin = 5;
const int kCalValueEepromAddress = 0;

HX711_ADC gLoadCell(kHx711DoutPin, kHx711SckPin);

unsigned long gLastPrintTimeMs = 0;

void setup()
{
  Serial.begin(57600);
  delay(10);

  gLoadCell.begin();

  // gLoadCell.setReverseOutput();

  unsigned long tStabilizingTimeMs = 2000;
  bool tPerformTare = true;

  gLoadCell.start(tStabilizingTimeMs, tPerformTare);
  gLoadCell.setSamplesInUse(4);

  if (gLoadCell.getTareTimeoutFlag() || gLoadCell.getSignalTimeoutFlag())
  {
    while (1);
  }

#if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(512);
#endif

  float tSavedCalibrationValue = 0.0f;
  EEPROM.get(kCalValueEepromAddress, tSavedCalibrationValue);

  if (tSavedCalibrationValue == 0.0f)
  {
    while (1);
  }

  gLoadCell.setCalFactor(tSavedCalibrationValue);

  while (!gLoadCell.update())
  {
  }
}

void loop()
{
  static bool tIsDataReady = false;
  const unsigned long tSerialPrintIntervalMs = 0;

  if (gLoadCell.update())
  {
    tIsDataReady = true;
  }

  if (tIsDataReady)
  {
    if (millis() - gLastPrintTimeMs >= tSerialPrintIntervalMs)
    {
      float tMeasuredMassG = gLoadCell.getData();

      Serial.print(millis());
      Serial.print(",");
      Serial.println(tMeasuredMassG, 3);

      tIsDataReady = false;
      gLastPrintTimeMs = millis();
    }
  }

  if (Serial.available() > 0)
  {
    char tInputChar = Serial.read();

    if (tInputChar == 't')
    {
      gLoadCell.tareNoDelay();
    }
  }
}