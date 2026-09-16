#include <ESP32Servo.h>

Servo miServo;

// Pines
const int pinServo = 18;
const int pinBoton = 4;

// Variables
bool estadoBoton = HIGH;
bool ultimoEstado = HIGH;
bool posicion = false;

void setup() {
  Serial.begin(115200);

  // Configurar servo
  miServo.setPeriodHertz(50);
  miServo.attach(pinServo, 500, 2400);
  miServo.write(110);

  // Configurar botón
  pinMode(pinBoton, INPUT_PULLUP);
}

void loop() {
  estadoBoton = digitalRead(pinBoton);

  // Detectar pulsación
  if (ultimoEstado == HIGH && estadoBoton == LOW) {
    Serial.println("Botón presionado");

    posicion = !posicion;

    if (posicion) {
      miServo.write(165);
    } else {
      miServo.write(110);
    }

    delay(300); // anti-rebote
  }

  // ESTA LÍNEA DEBE ESTAR DENTRO DEL LOOP
  ultimoEstado = estadoBoton;
}