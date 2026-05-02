#include <Adafruit_MPU6050.h>

#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 bulunamadi!");
    while (true) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("AX,AY,AZ,GX,GY,GZ");
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(&accel, &gyro, &temp);

  Serial.print(accel.acceleration.x, 4);
  Serial.print(",");
  Serial.print(accel.acceleration.y, 4);
  Serial.print(",");
  Serial.print(accel.acceleration.z, 4);
  Serial.print(",");
  Serial.print(gyro.gyro.x, 4);
  Serial.print(",");
  Serial.print(gyro.gyro.y, 4);
  Serial.print(",");
  Serial.println(gyro.gyro.z, 4);

  delay(20); // yaklaşık 50 Hz
}