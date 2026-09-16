#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

void setup() {
  Serial.begin(115200);

  Wire.begin(23, 22); // SDA, SCL

  if (!sensor.init()) {
    Serial.println("Sensor no detectado 😢");
    while (1);
  }

  sensor.setModel(MS5837::MS5837_02BA); 

  Serial.println("Sensor listo 👍");
}

void loop() {
  sensor.read();

  Serial.print("Presion: ");
  Serial.print(sensor.pressure());
  Serial.println(" mbar");

  Serial.print("Temperatura: ");
  Serial.print(sensor.temperature());
  Serial.println(" °C");

  Serial.println("-------------------");

  delay(1000);
}
