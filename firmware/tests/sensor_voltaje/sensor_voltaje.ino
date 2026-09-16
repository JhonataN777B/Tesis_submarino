#include <Wire.h>
#include <Adafruit_INA3221.h>

Adafruit_INA3221 ina3221;

void setup() {

  Serial.begin(115200);

  // SDA, SCL
  Wire.begin(25, 26);

  if (!ina3221.begin()) {

    Serial.println("No se encontro INA3221");

    while (1);
  }

  Serial.println("INA3221 listo");
}

void loop() {

  // Canal 1
  float voltage1 = ina3221.getBusVoltage(1);
  float current1 = ina3221.getCurrentAmps(1);

  // Canal 2
  float voltage2 = ina3221.getBusVoltage(2);
  float current2 = ina3221.getCurrentAmps(2);

  // Canal 3
  float voltage3 = ina3221.getBusVoltage(3);
  float current3 = ina3221.getCurrentAmps(3);

  Serial.println("==============");

  Serial.print("Canal 1 Voltaje: ");
  Serial.print(voltage1);
  Serial.println(" V");

  Serial.print("Canal 1 Corriente: ");
  Serial.print(current1);
  Serial.println(" A");

  Serial.println();

  Serial.print("Canal 2 Voltaje: ");
  Serial.print(voltage2);
  Serial.println(" V");

  Serial.print("Canal 2 Corriente: ");
  Serial.print(current2);
  Serial.println(" A");

  Serial.println();

  Serial.print("Canal 3 Voltaje: ");
  Serial.print(voltage3);
  Serial.println(" V");

  Serial.print("Canal 3 Corriente: ");
  Serial.print(current3);
  Serial.println(" A");

  Serial.println();

  delay(1000);
}