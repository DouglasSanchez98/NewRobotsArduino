#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

void calibrateMPU6050(int samples = 100) {
  Serial.println("Calibrando sensores...");

  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;

  sensors_event_t a, g, temp;

  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sum_ax += a.acceleration.x;
    sum_ay += a.acceleration.y;
    sum_az += a.acceleration.z - 9.81;  // Restar gravedad
    sum_gx += g.gyro.x;
    sum_gy += g.gyro.y;
    sum_gz += g.gyro.z;
    delay(10);
  }

  accel_offset_x = sum_ax / samples;
  accel_offset_y = sum_ay / samples;
  accel_offset_z = sum_az / samples;
  gyro_offset_x = sum_gx / samples;
  gyro_offset_y = sum_gy / samples;
  gyro_offset_z = sum_gz / samples;

  Serial.println("Calibración completada.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!mpu.begin()) {
    Serial.println("Error: No se encontró el MPU6050");
    while (1) delay(10);
  }

  Serial.println("MPU6050 encontrado!");

  // Configuración de rangos
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Calibración
  calibrateMPU6050();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Aceleración X: ");
  Serial.print(a.acceleration.x - accel_offset_x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y - accel_offset_y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z - accel_offset_z);
  Serial.println(" m/s^2");

  Serial.print("Giroscopio X: ");
  Serial.print(g.gyro.x - gyro_offset_x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y - gyro_offset_y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z - gyro_offset_z);
  Serial.println(" rad/s");

  Serial.print("Temperatura: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.println();
  delay(500);
}
