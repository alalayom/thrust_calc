#include <Adafruit_MPU6050.h>

#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_MPU6050 gMpu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!gMpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not found!");
    while (true) {
      delay(1000);
    }
  }

  gMpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  gMpu.setGyroRange(MPU6050_RANGE_500_DEG);
  gMpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("AX,AY,AZ,GX,GY,GZ");
}

void loop() {
  sensors_event_t tAccel;
  sensors_event_t tGyro;
  sensors_event_t tTemp;

  gMpu.getEvent(&tAccel, &tGyro, &tTemp);

  Serial.print(tAccel.acceleration.x, 4);
  Serial.print(",");
  Serial.print(tAccel.acceleration.y, 4);
  Serial.print(",");
  Serial.print(tAccel.acceleration.z, 4);
  Serial.print(",");
  Serial.print(tGyro.gyro.x, 4);
  Serial.print(",");
  Serial.print(tGyro.gyro.y, 4);
  Serial.print(",");
  Serial.println(tGyro.gyro.z, 4);

  delay(20);
}