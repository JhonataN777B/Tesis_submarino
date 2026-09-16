/*
  Control unificado para ESP32-S3
  Control por Monitor Serial a 115200 baudios, fin de linea: Nueva linea.

  Librerias necesarias (Administrador de bibliotecas Arduino):
  - ESP32Servo
  - Adafruit NeoPixel
  - MS5837 (Blue Robotics / Rob Tillaart compatible con MS5837.h)
  - Adafruit MPU6050 y Adafruit Unified Sensor
*/

#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MS5837.h"

// ------------------------- Pines asignados -------------------------
constexpr uint8_t PIN_ACS712 = 1;
constexpr uint8_t PIN_BATERIA = 10;
constexpr uint8_t PIN_I2C_SDA = 8;
constexpr uint8_t PIN_I2C_SCL = 9;
constexpr uint8_t PIN_SERVO_PINZA = 2;
constexpr uint8_t PIN_ESC = 15;
constexpr uint8_t PIN_BOMBA_AI = 4;
constexpr uint8_t PIN_BOMBA_AD = 5;
constexpr uint8_t PIN_BOMBA_BI = 11;
constexpr uint8_t PIN_BOMBA_BD = 12;
constexpr uint8_t PIN_AIRE_1 = 6;
constexpr uint8_t PIN_AIRE_2 = 7;
constexpr uint8_t PIN_ELECTROVALVULA = 13;
constexpr uint8_t PIN_NEOPIXEL = 21;
constexpr uint8_t NUM_PIXELS = 16;  // Cambie este valor si su aro tiene otra cantidad.

// Divisor de bateria: 30 kOhm arriba y 10 kOhm abajo.
constexpr float FACTOR_DIVISOR_BATERIA = 4.2318f;
constexpr float ADC_REFERENCIA = 3.3f;
// ACS712 (20A) alimentado a 5V con divisor 10k/10k a la salida
constexpr float FACTOR_DIVISOR_ACS712 = 2.0f;      // Divisor 10k/10k (divide por 2)
constexpr float ACS712_CERO_V = 2.5f;              // Reposo nativo a 5V (5V / 2)
constexpr float ACS712_SENSIBILIDAD_V_A = 0.100f; // 100 mV/A para el modelo ACS712-20B

Servo servoPinza;
Servo esc;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;
MS5837 presion;

bool mpuDisponible = false;
bool presionDisponible = false;
int servoAngulo = 110;
int escMicrosegundos = 1000;

enum ModoLed { LED_APAGADO, LED_ERROR, LED_CONFIG, LED_LISTO, LED_OPERANDO, LED_BATERIA };
ModoLed modoLed = LED_APAGADO;
unsigned long ultimoLedMs = 0;
uint16_t pixelConfiguracion = 0;
int brilloOperacion = 5;
int pasoBrillo = 1;
bool estadoParpadeo = false;

void apagarBombasAgua();
void apagarAire();
void apagarTodo();
void procesarComando(char *comando);
void actualizarLeds();
void colorTodos(uint8_t r, uint8_t g, uint8_t b);
void leerSensores();
void imprimirAyuda();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(PIN_BOMBA_AI, OUTPUT);
  pinMode(PIN_BOMBA_AD, OUTPUT);
  pinMode(PIN_BOMBA_BI, OUTPUT);
  pinMode(PIN_BOMBA_BD, OUTPUT);
  pinMode(PIN_AIRE_1, OUTPUT);
  pinMode(PIN_AIRE_2, OUTPUT);
  pinMode(PIN_ELECTROVALVULA, OUTPUT);
  analogReadResolution(12);
  apagarTodo();

  servoPinza.setPeriodHertz(50);
  servoPinza.attach(PIN_SERVO_PINZA, 500, 2400);
  servoPinza.write(servoAngulo);

  // El ESC debe encender siempre con pulso minimo por seguridad.
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, 1000, 2000);
  esc.writeMicroseconds(escMicrosegundos);

  pixels.begin();
  pixels.setBrightness(40);
  pixels.clear();
  pixels.show();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  mpuDisponible = mpu.begin();
  if (mpuDisponible) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  presionDisponible = presion.init();
  if (presionDisponible) presion.setModel(MS5837::MS5837_02BA);

  Serial.println("\nControl submarino ESP32-S3 listo.");
  Serial.printf("MPU6050: %s | MS5837: %s\n", mpuDisponible ? "OK" : "NO detectado", presionDisponible ? "OK" : "NO detectado");
  imprimirAyuda();
}

void loop() {
  static char linea[64];
  static uint8_t longitud = 0;

  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      linea[longitud] = '\0';
      if (longitud > 0) procesarComando(linea);
      longitud = 0;
    } else if (longitud < sizeof(linea) - 1) {
      linea[longitud++] = c;
    }
  }
  actualizarLeds();
}

void procesarComando(char *comando) {
  for (char *p = comando; *p; ++p) *p = tolower(*p);

  if (!strcmp(comando, "ayuda") || !strcmp(comando, "help")) imprimirAyuda();
  else if (!strcmp(comando, "estado") || !strcmp(comando, "sensores")) leerSensores();
  else if (!strcmp(comando, "stop") || !strcmp(comando, "parar")) {
    apagarTodo();
    escMicrosegundos = 1000;
    esc.writeMicroseconds(escMicrosegundos);
    Serial.println("Todo detenido; ESC en minimo.");
  }
  // Bombas de agua: la combinacion se conserva de los sketches originales.
  else if (!strcmp(comando, "adelante") || !strcmp(comando, "a")) {
    apagarBombasAgua(); digitalWrite(PIN_BOMBA_AI, HIGH); digitalWrite(PIN_BOMBA_AD, HIGH); Serial.println("Bombas de agua: adelante.");
  } else if (!strcmp(comando, "derecha") || !strcmp(comando, "d")) {
    apagarBombasAgua(); digitalWrite(PIN_BOMBA_AI, HIGH); digitalWrite(PIN_BOMBA_BI, HIGH); Serial.println("Bombas de agua: giro derecha.");
  } else if (!strcmp(comando, "izquierda") || !strcmp(comando, "i")) {
    apagarBombasAgua(); digitalWrite(PIN_BOMBA_AD, HIGH); digitalWrite(PIN_BOMBA_BD, HIGH); Serial.println("Bombas de agua: giro izquierda.");
  } else if (!strcmp(comando, "roll_derecha") || !strcmp(comando, "rd")) {
    apagarBombasAgua(); digitalWrite(PIN_BOMBA_AD, HIGH); digitalWrite(PIN_BOMBA_BI, HIGH); Serial.println("Bombas de agua: roll derecha.");
  } else if (!strcmp(comando, "roll_izquierda") || !strcmp(comando, "ri")) {
    apagarBombasAgua(); digitalWrite(PIN_BOMBA_AI, HIGH); digitalWrite(PIN_BOMBA_BD, HIGH); Serial.println("Bombas de agua: roll izquierda.");
  } else if (!strcmp(comando, "agua_off")) {
    apagarBombasAgua(); Serial.println("Bombas de agua apagadas.");
  }
  else if (!strcmp(comando, "aire1_on")) { digitalWrite(PIN_AIRE_1, HIGH); Serial.println("Bomba de aire 1 encendida."); }
  else if (!strcmp(comando, "aire1_off")) { digitalWrite(PIN_AIRE_1, LOW); Serial.println("Bomba de aire 1 apagada."); }
  else if (!strcmp(comando, "aire2_on")) { digitalWrite(PIN_AIRE_2, HIGH); Serial.println("Bomba de aire 2 encendida."); }
  else if (!strcmp(comando, "aire2_off")) { digitalWrite(PIN_AIRE_2, LOW); Serial.println("Bomba de aire 2 apagada."); }
  else if (!strcmp(comando, "aire_on")) { digitalWrite(PIN_AIRE_1, HIGH); digitalWrite(PIN_AIRE_2, HIGH); Serial.println("Bombas de aire encendidas."); }
  else if (!strcmp(comando, "aire_off")) { apagarAire(); Serial.println("Bombas de aire apagadas."); }
  else if (!strcmp(comando, "valvula_on")) { digitalWrite(PIN_ELECTROVALVULA, HIGH); Serial.println("Electrovalvula encendida."); }
  else if (!strcmp(comando, "valvula_off")) { digitalWrite(PIN_ELECTROVALVULA, LOW); Serial.println("Electrovalvula apagada."); }
  else if (!strncmp(comando, "servo ", 6)) {
    int angulo = atoi(comando + 6);
    if (angulo >= 0 && angulo <= 180) { servoAngulo = angulo; servoPinza.write(servoAngulo); Serial.printf("Pinza: %d grados.\n", servoAngulo); }
    else Serial.println("Angulo invalido: use servo 0..180.");
  } else if (!strcmp(comando, "abrir")) { servoAngulo = 165; servoPinza.write(servoAngulo); Serial.println("Pinza abierta."); }
  else if (!strcmp(comando, "cerrar")) { servoAngulo = 110; servoPinza.write(servoAngulo); Serial.println("Pinza cerrada."); }
  else if (!strncmp(comando, "esc ", 4)) {
    int pulso = atoi(comando + 4);
    if (pulso >= 1000 && pulso <= 2000) { escMicrosegundos = pulso; esc.writeMicroseconds(escMicrosegundos); Serial.printf("ESC: %d us.\n", escMicrosegundos); }
    else Serial.println("Pulso invalido: use esc 1000..2000.");
  } else if (!strcmp(comando, "esc_off")) { escMicrosegundos = 1000; esc.writeMicroseconds(escMicrosegundos); Serial.println("ESC en minimo."); }
  else if (!strcmp(comando, "led_off")) { modoLed = LED_APAGADO; pixels.clear(); pixels.show(); }
  else if (!strcmp(comando, "error")) { modoLed = LED_ERROR; Serial.println("LED: error."); }
  else if (!strcmp(comando, "config")) { modoLed = LED_CONFIG; Serial.println("LED: configurando."); }
  else if (!strcmp(comando, "listo")) { modoLed = LED_LISTO; colorTodos(0, 120, 255); }
  else if (!strcmp(comando, "operando")) { modoLed = LED_OPERANDO; Serial.println("LED: operando."); }
  else if (!strcmp(comando, "bateria")) { modoLed = LED_BATERIA; colorTodos(255, 80, 0); }
  else Serial.println("Comando no valido. Escriba ayuda.");
}

void apagarBombasAgua() { digitalWrite(PIN_BOMBA_AI, LOW); digitalWrite(PIN_BOMBA_AD, LOW); digitalWrite(PIN_BOMBA_BI, LOW); digitalWrite(PIN_BOMBA_BD, LOW); }
void apagarAire() { digitalWrite(PIN_AIRE_1, LOW); digitalWrite(PIN_AIRE_2, LOW); }
void apagarTodo() { apagarBombasAgua(); apagarAire(); digitalWrite(PIN_ELECTROVALVULA, LOW); }

void leerSensores() {
  float vAcs = analogRead(PIN_ACS712) * ADC_REFERENCIA / 4095.0f;
  float vBateriaPin = analogRead(PIN_BATERIA) * ADC_REFERENCIA / 4095.0f;
  Serial.printf("ACS712: %.3f V, corriente estimada: %.2f A\n", vAcs, (vAcs - ACS712_CERO_V) / ACS712_SENSIBILIDAD_V_A);
  Serial.printf("Bateria: %.2f V (pin ADC: %.3f V)\n", vBateriaPin * FACTOR_DIVISOR_BATERIA, vBateriaPin);
  if (presionDisponible) { presion.read(); Serial.printf("MS5837: %.2f mbar, %.2f C\n", presion.pressure(), presion.temperature()); }
  else Serial.println("MS5837 no disponible.");
  if (mpuDisponible) { sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t); Serial.printf("MPU6050 accel[m/s2]: X %.2f Y %.2f Z %.2f | gyro[rad/s]: X %.2f Y %.2f Z %.2f\n", a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z); }
  else Serial.println("MPU6050 no disponible.");
}

void actualizarLeds() {
  unsigned long ahora = millis();
  if (modoLed == LED_ERROR && ahora - ultimoLedMs >= 400) { ultimoLedMs = ahora; estadoParpadeo = !estadoParpadeo; if (estadoParpadeo) colorTodos(255, 0, 0); else { pixels.clear(); pixels.show(); } }
  else if (modoLed == LED_CONFIG && ahora - ultimoLedMs >= 80) { ultimoLedMs = ahora; pixels.clear(); pixels.setPixelColor(pixelConfiguracion++ % NUM_PIXELS, pixels.Color(255, 120, 0)); pixels.show(); }
  else if (modoLed == LED_OPERANDO && ahora - ultimoLedMs >= 20) { ultimoLedMs = ahora; pixels.setBrightness(brilloOperacion); colorTodos(0, 0, 255); brilloOperacion += pasoBrillo; if (brilloOperacion >= 40 || brilloOperacion <= 5) pasoBrillo = -pasoBrillo; }
}

void colorTodos(uint8_t r, uint8_t g, uint8_t b) { for (uint16_t i = 0; i < NUM_PIXELS; ++i) pixels.setPixelColor(i, pixels.Color(r, g, b)); pixels.show(); }

void imprimirAyuda() {
  Serial.println("Comandos: ayuda | estado | stop");
  Serial.println("Agua: adelante/a, derecha/d, izquierda/i, roll_derecha/rd, roll_izquierda/ri, agua_off");
  Serial.println("Aire: aire_on, aire_off, aire1_on/off, aire2_on/off | valvula_on/off");
  Serial.println("Pinza: abrir, cerrar, servo 0..180 | Propulsor: esc 1000..2000, esc_off");
  Serial.println("LED: error, config, listo, operando, bateria, led_off");
}
