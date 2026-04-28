#include <HX711_ADC.h>

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

const int kHx711DoutPin = 4;
const int kHx711SckPin = 5;
const int kCalValueEepromAddress = 0;

HX711_ADC gLoadCell(kHx711DoutPin, kHx711SckPin);

void run_calibration();
void change_saved_cal_factor();

void setup()
{
  Serial.begin(57600);
  delay(10);

  Serial.println();
  Serial.println("Starting...");

  gLoadCell.begin();

  // gLoadCell.setReverseOutput();

  unsigned long tStabilizingTimeMs = 10000;
  bool tPerformTare = true;

  gLoadCell.start(tStabilizingTimeMs, tPerformTare);
  gLoadCell.setSamplesInUse(4);

  if (gLoadCell.getTareTimeoutFlag() || gLoadCell.getSignalTimeoutFlag())
  {
    Serial.println("Timeout detected. Check MCU to HX711 wiring and pin assignments.");
    while (1);
  }
  else
  {
    gLoadCell.setCalFactor(1.0);
    Serial.println("Startup complete.");
  }

  while (!gLoadCell.update())
  {
  }

  run_calibration();
}

void loop()
{
  static bool tIsDataReady = false;
  const unsigned long tSerialPrintIntervalMs = 200;

  if (gLoadCell.update())
  {
    tIsDataReady = true;
  }

  if (tIsDataReady)
  {
    static unsigned long tLastSerialPrintTimeMs = 0;

    if (millis() - tLastSerialPrintTimeMs >= tSerialPrintIntervalMs)
    {
      float tMeasuredMassG = gLoadCell.getData();

      Serial.print("Load cell output value: ");
      Serial.println(tMeasuredMassG);

      tIsDataReady = false;
      tLastSerialPrintTimeMs = millis();
    }
  }

  if (Serial.available() > 0)
  {
    char tInputChar = Serial.read();

    if (tInputChar == 't')
    {
      gLoadCell.tareNoDelay();
    }
    else if (tInputChar == 'r')
    {
      run_calibration();
    }
    else if (tInputChar == 'c')
    {
      change_saved_cal_factor();
    }
  }

  if (gLoadCell.getTareStatus() == true)
  {
    Serial.println("Tare complete.");
  }
}

void run_calibration()
{
  Serial.println("***");
  Serial.println("Calibration started.");
  Serial.println("Place the load cell on a level and stable surface.");
  Serial.println("Remove any load from the load cell.");
  Serial.println("Send 't' from Serial Monitor to set the tare offset.");

  bool tShouldResume = false;

  while (tShouldResume == false)
  {
    gLoadCell.update();

    if (Serial.available() > 0)
    {
      char tInputChar = Serial.read();

      if (tInputChar == 't')
      {
        gLoadCell.tareNoDelay();
      }
    }

    if (gLoadCell.getTareStatus() == true)
    {
      Serial.println("Tare complete.");
      tShouldResume = true;
    }
  }

  Serial.println("Place the known mass on the load cell.");
  Serial.println("Then send the mass value from Serial Monitor, for example: 100.0");

  float tKnownMassG = 0.0f;
  tShouldResume = false;

  while (tShouldResume == false)
  {
    gLoadCell.update();

    if (Serial.available() > 0)
    {
      tKnownMassG = Serial.parseFloat();

      if (tKnownMassG != 0.0f)
      {
        Serial.print("Known mass: ");
        Serial.println(tKnownMassG);
        tShouldResume = true;
      }
    }
  }

  gLoadCell.refreshDataSet();
  float tNewCalibrationValue = gLoadCell.getNewCalibration(tKnownMassG);

  Serial.print("New calibration value: ");
  Serial.println(tNewCalibrationValue);
  Serial.println("Use this value as the calibration factor in the project sketch.");

  Serial.print("Save this value to EEPROM address ");
  Serial.print(kCalValueEepromAddress);
  Serial.println("? y/n");

  tShouldResume = false;

  while (tShouldResume == false)
  {
    if (Serial.available() > 0)
    {
      char tInputChar = Serial.read();

      if (tInputChar == 'y')
      {
#if defined(ESP8266) || defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(kCalValueEepromAddress, tNewCalibrationValue);
#if defined(ESP8266) || defined(ESP32)
        EEPROM.commit();
#endif

        float tSavedCalibrationValue = 0.0f;
        EEPROM.get(kCalValueEepromAddress, tSavedCalibrationValue);

        Serial.print("Value ");
        Serial.print(tSavedCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(kCalValueEepromAddress);

        tShouldResume = true;
      }
      else if (tInputChar == 'n')
      {
        Serial.println("Value was not saved to EEPROM.");
        tShouldResume = true;
      }
    }
  }

  Serial.println("Calibration finished.");
  Serial.println("***");
  Serial.println("Send 'r' to recalibrate.");
  Serial.println("Send 'c' to manually change the calibration value.");
  Serial.println("***");
}

void change_saved_cal_factor()
{
  float tCurrentCalibrationValue = gLoadCell.getCalFactor();
  bool tShouldResume = false;

  Serial.println("***");
  Serial.print("Current calibration value: ");
  Serial.println(tCurrentCalibrationValue);
  Serial.println("Send the new calibration value from Serial Monitor, for example: 696.0");

  float tNewCalibrationValue = 0.0f;

  while (tShouldResume == false)
  {
    if (Serial.available() > 0)
    {
      tNewCalibrationValue = Serial.parseFloat();

      if (tNewCalibrationValue != 0.0f)
      {
        Serial.print("New calibration value: ");
        Serial.println(tNewCalibrationValue);

        gLoadCell.setCalFactor(tNewCalibrationValue);
        tShouldResume = true;
      }
    }
  }

  tShouldResume = false;

  Serial.print("Save this value to EEPROM address ");
  Serial.print(kCalValueEepromAddress);
  Serial.println("? y/n");

  while (tShouldResume == false)
  {
    if (Serial.available() > 0)
    {
      char tInputChar = Serial.read();

      if (tInputChar == 'y')
      {
#if defined(ESP8266) || defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(kCalValueEepromAddress, tNewCalibrationValue);
#if defined(ESP8266) || defined(ESP32)
        EEPROM.commit();
#endif

        float tSavedCalibrationValue = 0.0f;
        EEPROM.get(kCalValueEepromAddress, tSavedCalibrationValue);

        Serial.print("Value ");
        Serial.print(tSavedCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(kCalValueEepromAddress);

        tShouldResume = true;
      }
      else if (tInputChar == 'n')
      {
        Serial.println("Value was not saved to EEPROM.");
        tShouldResume = true;
      }
    }
  }

  Serial.println("Calibration value update finished.");
  Serial.println("***");
}