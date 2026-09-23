/*
  Control unificado para ESP32-S3
  Control por Monitor Serial a 115200 baudios, fin de linea: Nueva linea.

  Librerias necesarias (Administrador de bibliotecas Arduino):
  - ESP32Servo
  - Adafruit NeoPixel
  - MS5837 (Blue Robotics / Rob Tillaart compatible con MS5837.h)
  - IMU MPU6500/MPU9250 por I2C directo (sin libreria MPU)
*/

#include <Wire.h>
#include <math.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
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

// true si el GPIO15 maneja la entrada del ESC mediante NPN de emisor comun.
// El PWM de hardware se invierte para compensar la inversion del transistor.
constexpr bool ESC_USA_TRANSISTOR_NPN = true;
constexpr bool ESC_REVERSIBLE = true;
constexpr unsigned long TIEMPO_ARMADO_ESC_MS = 3000;
constexpr int ESC_PULSO_MINIMO = 1000;
constexpr int ESC_PULSO_NEUTRO = ESC_REVERSIBLE ? 1500 : 1000;
constexpr int ESC_PULSO_MAXIMO = 2000;
constexpr uint32_t PERIODO_ESC_US = 20000;
constexpr uint8_t ESC_PWM_RESOLUCION = 14;
constexpr uint32_t ESC_PWM_DUTY_MAX = (1UL << ESC_PWM_RESOLUCION) - 1;

// Divisor de bateria: 30 kOhm arriba y 10 kOhm abajo.
constexpr float FACTOR_DIVISOR_BATERIA_PREDETERMINADO = 4.2318f;
// ACS712 (20A) alimentado a 5V con divisor 10k/10k a la salida
constexpr float FACTOR_DIVISOR_ACS712 = 2.0f;      // Divisor 10k/10k (divide por 2)
constexpr float ACS712_SENSIBILIDAD_V_A = 0.100f; // 100 mV/A para el modelo ACS712-20B
constexpr float ACS712_ZONA_CERO_A = 0.08f;
constexpr uint16_t MUESTRAS_ADC = 128;
constexpr float DENSIDAD_AGUA_KG_M3 = 997.0f;
constexpr float GRAVEDAD_M_S2 = 9.80665f;
const uint8_t MODELO_MS5837 = MS5837::MS5837_02BA;

Servo servoPinza;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
MS5837 presion;
Preferences preferencias;

bool mpuDisponible = false;
bool magnetometroDisponible = false;
bool lecturaMagValida = false;
bool presionDisponible = false;
bool escPwmDisponible = false;
uint8_t direccionMpu = 0;
uint8_t whoAmIMpu = 0;
float ceroAcsPinV = 1.25f;
float factorDivisorBateria = FACTOR_DIVISOR_BATERIA_PREDETERMINADO;
float presionSuperficieMbar = 1013.25f;
bool superficieCalibrada = false;
float aceleracionX = 0, aceleracionY = 0, aceleracionZ = 0;
float giroX = 0, giroY = 0, giroZ = 0;
float magnetometroX = 0, magnetometroY = 0, magnetometroZ = 0;
float rollImu = 0, pitchImu = 0, rumboImu = 0;
float offsetGiroX = 0, offsetGiroY = 0, offsetGiroZ = 0;
float factorMagX = 1, factorMagY = 1, factorMagZ = 1;
unsigned long ultimaLecturaMpuMs = 0;
unsigned long ultimaSalidaSensoresMs = 0;
unsigned long ultimoCeroAcsMs = 0;
unsigned long escNeutroDesdeMs = 0;
float corrienteFiltradaA = 0;
bool filtroCorrienteInicializado = false;
uint8_t filasTabla = 0;
int servoAngulo = 110;
int escMicrosegundos = ESC_PULSO_NEUTRO;
bool orientacionInicializada = false;

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
float leerVoltajeAdcPromedio(uint8_t pin);
void calibrarCorriente();
void calibrarSuperficie();
void calibrarImu();
void actualizarMPU();
void actualizarCeroCorrienteAutomatico();
void imprimirEncabezadoTabla();
void escanearI2C();
bool hayDispositivoI2C(uint8_t direccion);
uint8_t leerRegistroI2C(uint8_t direccion, uint8_t registro);
bool leerRegistrosI2C(uint8_t direccion, uint8_t registro, uint8_t *buffer, uint8_t longitud);
bool escribirRegistroI2C(uint8_t direccion, uint8_t registro, uint8_t valor);
void iniciarMPU();
bool actualizarEsc();

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
  analogSetPinAttenuation(PIN_ACS712, ADC_11db);
  analogSetPinAttenuation(PIN_BATERIA, ADC_11db);
  preferencias.begin("submarino", false);
  factorDivisorBateria = preferencias.getFloat("factor_bat", FACTOR_DIVISOR_BATERIA_PREDETERMINADO);
  apagarTodo();

  servoPinza.setPeriodHertz(50);
  servoPinza.attach(PIN_SERVO_PINZA, 500, 2400);
  servoPinza.write(servoAngulo);

  pinMode(PIN_ESC, OUTPUT);
  digitalWrite(PIN_ESC, ESC_USA_TRANSISTOR_NPN ? HIGH : LOW);
  escPwmDisponible = ledcAttach(PIN_ESC, 50, ESC_PWM_RESOLUCION);
  if (!escPwmDisponible) {
    Serial.println("ERROR: no se pudo configurar PWM LEDC para el ESC.");
  } else if (!actualizarEsc()) {
    Serial.println("ERROR: LEDC no pudo escribir el pulso inicial del ESC.");
    escPwmDisponible = false;
  } else {
    Serial.printf("PWM ESC: LEDC 50 Hz, pulso %d us, reversible=%s, NPN=%s.\n", escMicrosegundos, ESC_REVERSIBLE ? "si" : "no", ESC_USA_TRANSISTOR_NPN ? "si" : "no");
  }

  pixels.begin();
  pixels.setBrightness(40);
  pixels.clear();
  pixels.show();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  escanearI2C();
  iniciarMPU();

  presionDisponible = presion.init();
  if (presionDisponible) {
    presion.setModel(MODELO_MS5837);
    calibrarSuperficie();
  }
  Serial.println("\nControl submarino ESP32-S3 listo.");
  Serial.printf("IMU: %s | AK8963: %s | MS5837: %s\n", mpuDisponible ? "OK" : "NO detectada", magnetometroDisponible ? "OK" : "NO detectado", presionDisponible ? "OK" : "NO detectado");
  Serial.printf("ESC: espere %lu ms en neutro para el armado.\n", TIEMPO_ARMADO_ESC_MS);
  delay(TIEMPO_ARMADO_ESC_MS);
  calibrarCorriente();  // Cero automático al arrancar con ESC en neutro.
  escNeutroDesdeMs = millis();
  imprimirAyuda();
  imprimirEncabezadoTabla();
  Serial.println("CONTROL_LISTO");
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
  actualizarMPU();
  actualizarCeroCorrienteAutomatico();
  if (millis() - ultimaSalidaSensoresMs >= 1000) {
    ultimaSalidaSensoresMs = millis();
    leerSensores();
  }
  actualizarLeds();
}

void procesarComando(char *comando) {
  for (char *p = comando; *p; ++p) *p = tolower(*p);

  if (!strcmp(comando, "conectar") || !strcmp(comando, "ping")) Serial.println("CONTROL_LISTO");
  else if (!strcmp(comando, "ayuda") || !strcmp(comando, "help")) imprimirAyuda();
  else if (!strcmp(comando, "estado") || !strcmp(comando, "sensores")) leerSensores();
  else if (!strcmp(comando, "calibrar_superficie")) calibrarSuperficie();
  else if (!strcmp(comando, "calibrar_imu")) { if (mpuDisponible) calibrarImu(); else Serial.println("IMU no disponible."); }
  else if (!strcmp(comando, "calibrar_brujula")) { if (magnetometroDisponible) Serial.println("Rumbo magnetico disponible, pero sin calibracion de hard/soft iron."); else Serial.println("AK8963 no detectado; sin brujula."); }
  else if (!strncmp(comando, "calibrar_bateria ", 17)) {
    float referencia = atof(comando + 17);
    float voltajePin = leerVoltajeAdcPromedio(PIN_BATERIA);
    if (referencia >= 5.0f && referencia <= 25.0f && voltajePin > 0.1f) {
      factorDivisorBateria = referencia / voltajePin;
      preferencias.putFloat("factor_bat", factorDivisorBateria);
      Serial.printf("Bateria calibrada: referencia %.3f V, ADC %.3f V, factor %.5f (guardado).\n", referencia, voltajePin, factorDivisorBateria);
    } else Serial.println("Uso: calibrar_bateria <voltaje medido con multimetro, 5..25 V>.");
  }
  else if (!strcmp(comando, "stop") || !strcmp(comando, "parar")) {
    apagarTodo();
    escMicrosegundos = ESC_PULSO_NEUTRO;
    actualizarEsc();
    escNeutroDesdeMs = millis();
    Serial.println("Todo detenido; ESC en neutro.");
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
    if (pulso >= ESC_PULSO_MINIMO && pulso <= ESC_PULSO_MAXIMO) {
      escMicrosegundos = pulso;
      if (!actualizarEsc()) { Serial.println("ERROR: no se pudo actualizar el PWM del ESC."); return; }
      escNeutroDesdeMs = escMicrosegundos == ESC_PULSO_NEUTRO ? millis() : 0;
      Serial.printf("ESC: %d us.\n", escMicrosegundos);
    }
    else Serial.println("Pulso invalido: use esc 1000..2000.");
  } else if (!strcmp(comando, "esc_off")) { escMicrosegundos = ESC_PULSO_NEUTRO; actualizarEsc(); escNeutroDesdeMs = millis(); Serial.println("ESC en neutro."); }
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

bool actualizarEsc() {
  if (!escPwmDisponible) return false;
  uint32_t tiempoAltoPinUs = ESC_USA_TRANSISTOR_NPN
      ? PERIODO_ESC_US - static_cast<uint32_t>(escMicrosegundos)
      : static_cast<uint32_t>(escMicrosegundos);
  uint32_t duty = (tiempoAltoPinUs * ESC_PWM_DUTY_MAX + PERIODO_ESC_US / 2)
      / PERIODO_ESC_US;
  return ledcWrite(PIN_ESC, duty);
}

float leerVoltajeAdcPromedio(uint8_t pin) {
  uint32_t suma = 0;
  for (uint16_t i = 0; i < MUESTRAS_ADC; ++i) suma += analogReadMilliVolts(pin);
  return (suma / static_cast<float>(MUESTRAS_ADC)) / 1000.0f;
}

void calibrarCorriente() {
  delay(20);
  ceroAcsPinV = leerVoltajeAdcPromedio(PIN_ACS712);
  corrienteFiltradaA = 0;
  filtroCorrienteInicializado = false;
  Serial.printf("ACS712 auto-cero: %.3f V con ESC en neutro.\n", ceroAcsPinV);
}

void calibrarSuperficie() {
  if (!presionDisponible) { Serial.println("MS5837 no disponible."); return; }
  presion.read();
  presionSuperficieMbar = presion.pressure();
  superficieCalibrada = true;
  Serial.printf("Superficie calibrada: %.2f mbar.\n", presionSuperficieMbar);
}

bool hayDispositivoI2C(uint8_t direccion) {
  Wire.beginTransmission(direccion);
  return Wire.endTransmission() == 0;
}

uint8_t leerRegistroI2C(uint8_t direccion, uint8_t registro) {
  uint8_t dato = 0xFF;
  leerRegistrosI2C(direccion, registro, &dato, 1);
  return dato;
}

bool leerRegistrosI2C(uint8_t direccion, uint8_t registro, uint8_t *buffer, uint8_t longitud) {
  Wire.beginTransmission(direccion);
  Wire.write(registro);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t recibidos = Wire.requestFrom(static_cast<int>(direccion), static_cast<int>(longitud));
  if (recibidos != longitud) { while (Wire.available()) Wire.read(); return false; }
  for (uint8_t i = 0; i < longitud; ++i) buffer[i] = Wire.read();
  return true;
}

bool escribirRegistroI2C(uint8_t direccion, uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(direccion);
  Wire.write(registro);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

void escanearI2C() {
  Serial.printf("I2C: SDA GPIO %u, SCL GPIO %u. Direcciones detectadas:", PIN_I2C_SDA, PIN_I2C_SCL);
  bool encontrado = false;
  for (uint8_t direccion = 1; direccion < 127; ++direccion) {
    if (hayDispositivoI2C(direccion)) { Serial.printf(" 0x%02X", direccion); encontrado = true; }
  }
  if (!encontrado) Serial.print(" ninguna");
  Serial.println();
}

void iniciarMPU() {
  const uint8_t direcciones[] = {0x68, 0x69};
  for (uint8_t direccion : direcciones) {
    if (!hayDispositivoI2C(direccion)) continue;
    uint8_t id = leerRegistroI2C(direccion, 0x75);
    Serial.printf("IMU en 0x%02X, WHO_AM_I=0x%02X. ", direccion, id);
    if (id != 0x70 && id != 0x71 && id != 0x73) { Serial.println("Identificador no reconocido."); continue; }
    direccionMpu = direccion;
    whoAmIMpu = id;
    if (!escribirRegistroI2C(direccionMpu, 0x6B, 0x00)) { Serial.println("No se pudo despertar la IMU."); continue; }
    delay(50);
    escribirRegistroI2C(direccionMpu, 0x1A, 0x03);
    escribirRegistroI2C(direccionMpu, 0x1C, 0x10);  // +/-8 g
    escribirRegistroI2C(direccionMpu, 0x1B, 0x08);  // +/-500 grados/s
    escribirRegistroI2C(direccionMpu, 0x37, 0x02);  // Bypass para probar AK8963
    delay(50);
    mpuDisponible = true;
    Serial.printf("IMU lista por I2C directo: %s (accel+gyro).\n", id == 0x70 ? "MPU6500" : (id == 0x71 ? "MPU9250" : "MPU9255"));

    // Seguir la inicializacion del sketch funcional del usuario: en algunos
    // clones el WIA no es estandar aunque el magnetometro responda en 0x0C.
    uint8_t idMag = leerRegistroI2C(0x0C, 0x00);
    bool respondeMag = hayDispositivoI2C(0x0C);
    if (respondeMag && escribirRegistroI2C(0x0C, 0x0A, 0x16)) {
      delay(50);  // Continuo 100 Hz, salida de 16 bits.
      magnetometroDisponible = true;
      Serial.printf("Magnetometro responde en 0x0C; WIA=0x%02X (no se usa para rechazar clones).\n", idMag);
    } else {
      Serial.printf("Magnetometro sin respuesta/inicializacion en 0x0C; WIA=0x%02X.\n", idMag);
    }
    return;
  }
  Serial.println("No se detecto MPU6500/MPU9250/MPU9255 en 0x68 ni 0x69.");
}

void actualizarMPU() {
  if (!mpuDisponible || millis() - ultimaLecturaMpuMs < 10) return;
  unsigned long ahora = millis();
  float dt = ultimaLecturaMpuMs == 0 ? 0.01f : (ahora - ultimaLecturaMpuMs) / 1000.0f;
  ultimaLecturaMpuMs = ahora;
  uint8_t datos[14] = {0};
  if (!leerRegistrosI2C(direccionMpu, 0x3B, datos, 14)) return;
  auto i16 = [](uint8_t hi, uint8_t lo) -> int16_t { return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo); };
  aceleracionX = i16(datos[0], datos[1]) / 4096.0f;
  aceleracionY = i16(datos[2], datos[3]) / 4096.0f;
  aceleracionZ = i16(datos[4], datos[5]) / 4096.0f;
  giroX = i16(datos[8], datos[9]) / 65.5f - offsetGiroX;
  giroY = i16(datos[10], datos[11]) / 65.5f - offsetGiroY;
  giroZ = i16(datos[12], datos[13]) / 65.5f - offsetGiroZ;
  float rollAcc = atan2f(aceleracionY, aceleracionZ) * 180.0f / PI;
  float pitchAcc = atan2f(-aceleracionX, sqrtf(aceleracionY * aceleracionY + aceleracionZ * aceleracionZ)) * 180.0f / PI;
  if (!orientacionInicializada) { rollImu = rollAcc; pitchImu = pitchAcc; orientacionInicializada = true; }
  else {
    rollImu = 0.98f * (rollImu + giroX * dt) + 0.02f * rollAcc;
    pitchImu = 0.98f * (pitchImu + giroY * dt) + 0.02f * pitchAcc;
  }
  if (magnetometroDisponible) {
    lecturaMagValida = false;
    // Leer el bloque directamente como en el sketch base funcional. Algunos
    // clones no implementan ST2 de forma fiable, así que no se descarta por ese byte.
    uint8_t mag[7] = {0};
    if (leerRegistrosI2C(0x0C, 0x03, mag, 7)) {
      magnetometroX = static_cast<int16_t>((static_cast<uint16_t>(mag[1]) << 8) | mag[0]) * 0.15f * factorMagX;
      magnetometroY = static_cast<int16_t>((static_cast<uint16_t>(mag[3]) << 8) | mag[2]) * 0.15f * factorMagY;
      magnetometroZ = static_cast<int16_t>((static_cast<uint16_t>(mag[5]) << 8) | mag[4]) * 0.15f * factorMagZ;
      rumboImu = atan2f(magnetometroY, magnetometroX) * 180.0f / PI;
      if (rumboImu < 0) rumboImu += 360.0f;
      lecturaMagValida = true;
    }
  }
}

void calibrarImu() {
  Serial.println("Deje la IMU quieta y nivelada durante la calibracion del giroscopio...");
  float sx = 0, sy = 0, sz = 0;
  uint16_t validas = 0;
  for (uint16_t i = 0; i < 250; ++i) {
    uint8_t d[14] = {0};
    if (leerRegistrosI2C(direccionMpu, 0x3B, d, 14)) {
      sx += static_cast<int16_t>((static_cast<uint16_t>(d[8]) << 8) | d[9]) / 65.5f;
      sy += static_cast<int16_t>((static_cast<uint16_t>(d[10]) << 8) | d[11]) / 65.5f;
      sz += static_cast<int16_t>((static_cast<uint16_t>(d[12]) << 8) | d[13]) / 65.5f;
      ++validas;
    }
    delay(4);
  }
  if (!validas) { Serial.println("No se pudieron leer muestras de IMU."); return; }
  offsetGiroX = sx / validas; offsetGiroY = sy / validas; offsetGiroZ = sz / validas;
  Serial.printf("Offsets gyro [deg/s]: X %.2f Y %.2f Z %.2f\n", offsetGiroX, offsetGiroY, offsetGiroZ);
}

void leerSensores() {
  // Refrescar la IMU al pedir estado; el bucle tambien la actualiza continuamente.
  actualizarMPU();
  float vAcs = leerVoltajeAdcPromedio(PIN_ACS712);
  float delta = vAcs - ceroAcsPinV;
  float corrienteCruda = delta * FACTOR_DIVISOR_ACS712 / ACS712_SENSIBILIDAD_V_A;
  float corrienteMuestra = fabsf(corrienteCruda);
  if (corrienteMuestra < ACS712_ZONA_CERO_A) corrienteMuestra = 0.0f;
  if (!filtroCorrienteInicializado) { corrienteFiltradaA = corrienteMuestra; filtroCorrienteInicializado = true; }
  else corrienteFiltradaA = 0.25f * corrienteMuestra + 0.75f * corrienteFiltradaA;
  float vBatPin = leerVoltajeAdcPromedio(PIN_BATERIA);
  float vBateria = vBatPin * factorDivisorBateria;
  float potenciaFiltradaW = vBateria * corrienteFiltradaA;
  if (filasTabla > 0 && filasTabla % 20 == 0) imprimirEncabezadoTabla();
  Serial.printf("%7.3f %7.2f %7.2f %7.2f ", vAcs, corrienteFiltradaA, potenciaFiltradaW, vBateria);
  if (presionDisponible) {
    presion.read();
    float profundidad = superficieCalibrada ? (presion.pressure() - presionSuperficieMbar) * 100.0f / (DENSIDAD_AGUA_KG_M3 * GRAVEDAD_M_S2) : 0;
    if (profundidad < 0) profundidad = 0;
    Serial.printf("%9.2f %7.2f %7.2f ", presion.pressure(), presion.temperature(), profundidad);
  } else Serial.print("       --      --      -- ");
  if (mpuDisponible) {
    Serial.printf("%6.2f %6.2f %6.2f %7.1f %7.1f %7.1f ", aceleracionX, aceleracionY, aceleracionZ, giroX, giroY, giroZ);
    if (magnetometroDisponible && lecturaMagValida) Serial.printf("%7.1f %7.1f %7.1f ", magnetometroX, magnetometroY, magnetometroZ);
    else Serial.print("     --      --      -- ");
    Serial.printf("%7.1f %7.1f ", rollImu, pitchImu);
    if (magnetometroDisponible && lecturaMagValida) Serial.printf("%7.1f\n", rumboImu);
    else Serial.println("     --");
  } else Serial.println("    --     --     --      --      --      --      --      --      --      --      --      --");
  ++filasTabla;
}

void imprimirEncabezadoTabla() {
  Serial.println("\n ACS[V]   I[A]    P[W]  Bat[V] Press[mbar] Temp[C] Depth[m]   Ax[g]   Ay[g]   Az[g] Gx[dps] Gy[dps] Gz[dps] Mx[uT] My[uT] Mz[uT] Roll[deg] Pitch[deg] Yaw[deg]");
  filasTabla = 0;
}

void actualizarCeroCorrienteAutomatico() {
  if (escMicrosegundos != ESC_PULSO_NEUTRO) { escNeutroDesdeMs = 0; return; }
  if (escNeutroDesdeMs == 0) escNeutroDesdeMs = millis();
  unsigned long ahora = millis();
  if (ahora - escNeutroDesdeMs < 2000 || ahora - ultimoCeroAcsMs < 1000) return;
  float lecturaReposo = leerVoltajeAdcPromedio(PIN_ACS712);
  ceroAcsPinV = 0.98f * ceroAcsPinV + 0.02f * lecturaReposo;
  ultimoCeroAcsMs = ahora;
  if (fabsf((lecturaReposo - ceroAcsPinV) * FACTOR_DIVISOR_ACS712 / ACS712_SENSIBILIDAD_V_A) < 0.35f) {
    corrienteFiltradaA *= 0.5f;
    if (corrienteFiltradaA < 0.03f) corrienteFiltradaA = 0;
  }
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
  Serial.println("Calibracion automatica: ACS712 al arrancar/en neutro; bateria guardada en memoria.");
  Serial.println("Calibracion opcional: calibrar_bateria <V multimetro>, calibrar_superficie, calibrar_imu");
  Serial.println("LED: error, config, listo, operando, bateria, led_off");
}
