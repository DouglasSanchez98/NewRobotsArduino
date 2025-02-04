#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Objeto para manejar el PCA9685
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// Valores ajustados para 50 Hz
unsigned int pos0 = 102;   // Pulso para 0° (~500 µs)
unsigned int pos180 = 537; // Pulso para 180° (~2500 µs)

// Función para mover el servo a un ángulo específico
void setServoAngle(uint8_t servoNum, float angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  // Convertir ángulo a rango de cuentas del PCA9685
  int pulse = map(angle, 0, 180, pos0, pos180);

  // Enviar pulso al servomotor en el canal especificado
  servos.setPWM(servoNum, 0, pulse);
}

void setup() {
  servos.begin();
  servos.setPWMFreq(50); // Frecuencia PWM de 50 Hz
}

void loop() {
  setServoAngle(0, 0);    // Mueve el servo en canal 0 a 0°
  delay(1000);
  setServoAngle(0, 90);   // Mueve el servo en canal 0 a 90°
  delay(1000);
  setServoAngle(0, 180);  // Mueve el servo en canal 0 a 180°
  delay(1000);
}
