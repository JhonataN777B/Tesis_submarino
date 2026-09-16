#define LED_PIN 18

void setup() {

  pinMode(LED_PIN, OUTPUT);
}

void loop() {

  // blink 1
  digitalWrite(LED_PIN, HIGH);
  delay(120);

  digitalWrite(LED_PIN, LOW);
  delay(120);

  // blink 2
  digitalWrite(LED_PIN, HIGH);
  delay(120);

  digitalWrite(LED_PIN, LOW);
  delay(700);
}