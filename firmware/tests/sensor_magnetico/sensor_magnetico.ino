const int pinSensor = 5;
const int pinLED = 21;

void setup() {
  Serial.begin(115200);

  pinMode(pinSensor, INPUT_PULLUP);
  pinMode(pinLED, OUTPUT);
}

void loop() {
  int estado = digitalRead(pinSensor);

  if (estado == LOW) {  // imán cerca
    Serial.println("Imán detectado");
    digitalWrite(pinLED, HIGH);
  } else {
    digitalWrite(pinLED, LOW);
  }
}