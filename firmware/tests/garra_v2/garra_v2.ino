#include <ESP32Servo.h>

Servo miServo;

// Pines
const int pinServo = 18;
const int pinPot = 32; // Pin analógico (ADC)

// Variables
int valorPot = 0;
int angulo = 0;

void setup() {
  Serial.begin(115200);

  // Configurar servo
  miServo.setPeriodHertz(50);
  miServo.attach(pinServo, 500, 2400);
}

void loop() {
  // Leer potenciómetro
  valorPot = analogRead(pinPot);

  // Convertir de 0-4095 a 0-180 grados
  angulo = map(valorPot, 0, 4095, 0, 180);

  // Mover servo
  miServo.write(angulo);

  // Mostrar datos
  Serial.print("Pot: ");
  Serial.print(valorPot);
  Serial.print(" | Angulo: ");
  Serial.println(angulo);

  delay(20);
}