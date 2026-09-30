/*
  Control unificado para ESP32-S3
  Control por Monitor Serial a 115200 baudios, fin de linea: Nueva linea.

  Librerias necesarias:
  - ESP32Servo
  - Adafruit NeoPixel
  - MS5837 (Blue Robotics / Rob Tillaart compatible con MS5837.h)
  - DFRobot_BNO055 (biblioteca oficial para SEN0374; se instala desde el ZIP de GitHub)
*/

#include <Wire.h>
#include <math.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include "MS5837.h"
#include "DFRobot_BNO055.h"

typedef DFRobot_BNO055_IIC BNO055;
BNO055 bno055_28(&Wire, 0x28);
BNO055 bno055_29(&Wire, 0x29);
BNO055 *sensorBno055 = nullptr;

// ------------------------- Pines asignados -------------------------
constexpr uint8_t PIN_ACS712 = 1;
constexpr uint8_t PIN_BATERIA = 10;
constexpr uint8_t PIN_I2C_SDA = 8;
constexpr uint8_t PIN_I2C_SCL = 9;
constexpr uint8_t BNO055_DIRECCION_1 = 0x28;
constexpr uint8_t BNO055_DIRECCION_2 = 0x29;
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
// Telemetria a 10 Hz: deja margen para las conversiones de 20 ms del MS5837.
constexpr unsigned long INTERVALO_TELEMETRIA_MS = 100;
constexpr int ESC_PULSO_MINIMO = 1000;
constexpr int ESC_PULSO_NEUTRO = ESC_REVERSIBLE ? 1500 : 1000;
constexpr int ESC_PULSO_MAXIMO = 2000;
constexpr uint32_t PERIODO_ESC_US = 20000;
constexpr uint8_t ESC_PWM_RESOLUCION = 14;
constexpr uint32_t ESC_PWM_DUTY_MAX = (1UL << ESC_PWM_RESOLUCION) - 1;

// Divisor de bateria: 30 kOhm arriba y 10 kOhm abajo.
constexpr float FACTOR_DIVISOR_BATERIA_PREDETERMINADO = 4.2318f;
constexpr float VOLTAJE_BATERIA_BAJO_V = 10.5f;
constexpr unsigned long TIEMPO_CONFIGURACION_LED_MS = 5000;
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

bool imuDisponible = false;
bool magnetometroDisponible = false;
bool lecturaImuValida = false;
bool lecturaMagValida = false;
bool presionDisponible = false;
bool escPwmDisponible = false;
bool sistemaOperando = false;
bool lecturaAcsValida = false;
bool lecturaBateriaValida = false;
bool lecturaPresionValida = false;
float voltajeBateriaActual = 0.0f;
uint8_t fallosLecturaImu = 0;
uint8_t fallosLecturaMag = 0;
uint8_t direccionBno055 = 0;
uint8_t calibracionSistema = 0;
uint8_t calibracionGiroscopio = 0;
uint8_t calibracionAcelerometro = 0;
uint8_t calibracionMagnetometro = 0;
float ceroAcsPinV = 1.25f;
float factorDivisorBateria = FACTOR_DIVISOR_BATERIA_PREDETERMINADO;
float presionSuperficieMbar = 1013.25f;
bool superficieCalibrada = false;
float aceleracionX = 0, aceleracionY = 0, aceleracionZ = 0;
float giroX = 0, giroY = 0, giroZ = 0;
float magnetometroX = 0, magnetometroY = 0, magnetometroZ = 0;
float rollImu = 0, pitchImu = 0, rumboImu = 0;
unsigned long ultimaLecturaImuMs = 0;
unsigned long ultimaSalidaSensoresMs = 0;
unsigned long ultimoCeroAcsMs = 0;
unsigned long escNeutroDesdeMs = 0;
float corrienteFiltradaA = 0;
bool filtroCorrienteInicializado = false;
uint8_t filasTabla = 0;
int servoAngulo = 110;
int escMicrosegundos = ESC_PULSO_NEUTRO;

enum ModoLed { LED_APAGADO, LED_ERROR, LED_CONFIG, LED_LISTO, LED_OPERANDO, LED_BATERIA };
ModoLed modoLed = LED_APAGADO;
unsigned long inicioSistemaMs = 0;
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
bool hayFallaSensores();
void colorTodos(uint8_t r, uint8_t g, uint8_t b);
void leerSensores();
void imprimirAyuda();
float leerVoltajeAdcPromedio(uint8_t pin);
void calibrarCorriente();
void calibrarSuperficie();
void imprimirCalibracionBNO055(const char *mensaje);
void actualizarBNO055();
void actualizarCeroCorrienteAutomatico();
void imprimirEncabezadoTabla();
void escanearI2C();
bool hayDispositivoI2C(uint8_t direccion);
void iniciarBNO055();
bool actualizarEsc();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  inicioSistemaMs = millis();

  pixels.begin();
  pixels.setBrightness(40);
  colorTodos(255, 120, 0);  // Configuracion visible desde el encendido.

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

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  escanearI2C();
  iniciarBNO055();

  presionDisponible = presion.init();
  if (presionDisponible) {
    presion.setModel(MODELO_MS5837);
    calibrarSuperficie();
  }
  Serial.println("\nControl submarino ESP32-S3 listo.");
  if (imuDisponible) Serial.printf("BNO055/SEN0374: OK en I2C 0x%02X | Magnetometro integrado: OK\n", direccionBno055);
  else Serial.println("BNO055/SEN0374: NO detectado en 0x28/0x29 | Magnetometro integrado: NO disponible.");
  Serial.printf("MS5837: %s\n", presionDisponible ? "OK" : "NO detectado");
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
  actualizarBNO055();
  actualizarCeroCorrienteAutomatico();
  if (millis() - ultimaSalidaSensoresMs >= INTERVALO_TELEMETRIA_MS) {
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
  else if (!strcmp(comando, "reiniciar") || !strcmp(comando, "reinicio") || !strcmp(comando, "reset")) {
    sistemaOperando = false;
    apagarTodo();
    escMicrosegundos = ESC_PULSO_NEUTRO;
    actualizarEsc();
    escNeutroDesdeMs = millis();
    Serial.println("Reinicio solicitado: actuadores apagados y ESC en neutro.");
    Serial.flush();
    delay(250);
    ESP.restart();
  }
  else if (!strcmp(comando, "calibrar_superficie")) calibrarSuperficie();
  else if (!strcmp(comando, "calibrar_imu")) { if (imuDisponible) imprimirCalibracionBNO055("Calibracion BNO055 (0=sin calibrar, 3=calibrado):"); else Serial.println("BNO055 no disponible."); }
  else if (!strcmp(comando, "calibrar_brujula")) {
    if (magnetometroDisponible) {
      Serial.println("Mueva el submarino lentamente en varias orientaciones, dibujando ochos, lejos de metales y corrientes del motor.");
      imprimirCalibracionBNO055("Estado de calibracion BNO055 (0=sin calibrar, 3=calibrado):");
    } else Serial.println("BNO055/magnetometro integrado no disponible.");
  }
  else if (!strncmp(comando, "calibrar_bateria ", 17)) {
    float referencia = atof(comando + 17);
    float voltajePin = leerVoltajeAdcPromedio(PIN_BATERIA);
    if (referencia >= 5.0f && referencia <= 25.0f && voltajePin > 0.1f) {
      factorDivisorBateria = referencia / voltajePin;
      preferencias.putFloat("factor_bat", factorDivisorBateria);
      Serial.printf("Bateria calibrada: referencia %.3f V, ADC %.3f V, factor %.5f (guardado).\n", referencia, voltajePin, factorDivisorBateria);
    } else Serial.println("Uso: calibrar_bateria <voltaje medido con multimetro, 5..25 V>.");
  }
  else if (!strcmp(comando, "iniciar")) {
    sistemaOperando = true;
    Serial.println("Control en modo OPERANDO.");
  }
  else if (!strcmp(comando, "stop") || !strcmp(comando, "parar")) {
    sistemaOperando = false;
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
    }
    else Serial.println("Pulso invalido: use esc 1000..2000.");
  } else if (!strcmp(comando, "esc_off")) { escMicrosegundos = ESC_PULSO_NEUTRO; actualizarEsc(); escNeutroDesdeMs = millis(); }
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

void escanearI2C() {
  Serial.printf("I2C: SDA GPIO %u, SCL GPIO %u. Direcciones detectadas:", PIN_I2C_SDA, PIN_I2C_SCL);
  bool encontrado = false;
  for (uint8_t direccion = 1; direccion < 127; ++direccion) {
    if (hayDispositivoI2C(direccion)) { Serial.printf(" 0x%02X", direccion); encontrado = true; }
  }
  if (!encontrado) Serial.print(" ninguna");
  Serial.println();
}

void iniciarBNO055() {
  const uint8_t direcciones[] = {BNO055_DIRECCION_1, BNO055_DIRECCION_2};
  BNO055 *candidatos[] = {&bno055_28, &bno055_29};
  for (uint8_t i = 0; i < 2; ++i) {
    uint8_t direccion = direcciones[i];
    if (!hayDispositivoI2C(direccion)) continue;
    Serial.printf("Probando biblioteca DFRobot_BNO055 en 0x%02X... ", direccion);
    BNO055::eStatus_t estado = candidatos[i]->begin();
    if (estado != BNO055::eStatusOK) {
      Serial.printf("inicio fallido (estado %d).\n", static_cast<int>(estado));
      continue;
    }

    sensorBno055 = candidatos[i];
    direccionBno055 = direccion;
    // begin() de DFRobot ya inicia el NDOF; se fija otra vez para dejar claro
    // que necesitamos orientacion fusionada con acelerometro, gyro y magnetometro.
    sensorBno055->setOprMode(BNO055::eOprModeNdof);
    imuDisponible = true;
    magnetometroDisponible = true; // El magnetometro va integrado en el BNO055.
    Serial.printf("BNO055 listo en 0x%02X, modo NDOF por biblioteca DFRobot.\n", direccionBno055);
    actualizarBNO055();
    imprimirCalibracionBNO055("Calibracion inicial BNO055 (0=sin calibrar, 3=calibrado):");
    return;
  }
  Serial.println("No se pudo iniciar la biblioteca DFRobot_BNO055 en 0x28 ni 0x29.");
}

void actualizarBNO055() {
  if (!imuDisponible || sensorBno055 == nullptr || millis() - ultimaLecturaImuMs < 10) return;
  ultimaLecturaImuMs = millis();

  // DFRobot entrega accel en mg, magnetometro en uT, gyro en dps y Euler en grados.
  BNO055::sAxisAnalog_t accel = sensorBno055->getAxis(BNO055::eAxisAcc);
  BNO055::sAxisAnalog_t mag = sensorBno055->getAxis(BNO055::eAxisMag);
  BNO055::sAxisAnalog_t gyro = sensorBno055->getAxis(BNO055::eAxisGyr);
  BNO055::sEulAnalog_t euler = sensorBno055->getEul();
  if (sensorBno055->lastOperateStatus != BNO055::eStatusOK) {
    lecturaImuValida = false;
    lecturaMagValida = false;
    if (fallosLecturaImu < 5) ++fallosLecturaImu;
    if (fallosLecturaMag < 5) ++fallosLecturaMag;
    return;
  }

  aceleracionX = accel.x / 1000.0f;
  aceleracionY = accel.y / 1000.0f;
  aceleracionZ = accel.z / 1000.0f;
  magnetometroX = mag.x;
  magnetometroY = mag.y;
  magnetometroZ = mag.z;
  giroX = gyro.x;
  giroY = gyro.y;
  giroZ = gyro.z;
  rumboImu = euler.head;
  rollImu = euler.roll;
  pitchImu = euler.pitch;
  while (rumboImu < 0.0f) rumboImu += 360.0f;
  while (rumboImu >= 360.0f) rumboImu -= 360.0f;

  lecturaImuValida = isfinite(aceleracionX) && isfinite(aceleracionY) && isfinite(aceleracionZ) &&
                     isfinite(giroX) && isfinite(giroY) && isfinite(giroZ) && isfinite(rollImu) && isfinite(pitchImu);
  lecturaMagValida = isfinite(magnetometroX) && isfinite(magnetometroY) && isfinite(magnetometroZ) && isfinite(rumboImu);
  if (lecturaImuValida) fallosLecturaImu = 0;
  if (lecturaMagValida) fallosLecturaMag = 0;

  BNO055::sRegCalibState_t calibracion = sensorBno055->getCalStatus();
  if (sensorBno055->lastOperateStatus == BNO055::eStatusOK) {
    calibracionSistema = calibracion.SYS;
    calibracionGiroscopio = calibracion.GYR;
    calibracionAcelerometro = calibracion.ACC;
    calibracionMagnetometro = calibracion.MAG;
  }
}

void imprimirCalibracionBNO055(const char *mensaje) {
  if (!imuDisponible || sensorBno055 == nullptr) { Serial.println("BNO055 no disponible."); return; }
  BNO055::sRegCalibState_t calibracion = sensorBno055->getCalStatus();
  if (sensorBno055->lastOperateStatus != BNO055::eStatusOK) {
    Serial.println("No se pudo leer el estado de calibracion del BNO055.");
    return;
  }
  calibracionSistema = calibracion.SYS;
  calibracionGiroscopio = calibracion.GYR;
  calibracionAcelerometro = calibracion.ACC;
  calibracionMagnetometro = calibracion.MAG;
  Serial.printf("%s Sistema=%u, gyro=%u, accel=%u, magnetometro=%u.\n",
                mensaje, calibracionSistema, calibracionGiroscopio,
                calibracionAcelerometro, calibracionMagnetometro);
}

void leerSensores() {
  // Refrescar la IMU al pedir estado; el bucle tambien la actualiza continuamente.
  actualizarBNO055();
  float vAcs = leerVoltajeAdcPromedio(PIN_ACS712);
  lecturaAcsValida = isfinite(vAcs) && vAcs > 0.05f && vAcs < 3.25f;
  float delta = vAcs - ceroAcsPinV;
  float corrienteCruda = delta * FACTOR_DIVISOR_ACS712 / ACS712_SENSIBILIDAD_V_A;
  float corrienteMuestra = fabsf(corrienteCruda);
  if (corrienteMuestra < ACS712_ZONA_CERO_A) corrienteMuestra = 0.0f;
  if (!filtroCorrienteInicializado) { corrienteFiltradaA = corrienteMuestra; filtroCorrienteInicializado = true; }
  else corrienteFiltradaA = 0.25f * corrienteMuestra + 0.75f * corrienteFiltradaA;
  float vBatPin = leerVoltajeAdcPromedio(PIN_BATERIA);
  float vBateria = vBatPin * factorDivisorBateria;
  voltajeBateriaActual = vBateria;
  lecturaBateriaValida = isfinite(vBatPin) && isfinite(vBateria) && vBatPin > 0.05f && vBateria < 25.0f;
  float potenciaFiltradaW = vBateria * corrienteFiltradaA;
  if (filasTabla > 0 && filasTabla % 20 == 0) imprimirEncabezadoTabla();
  Serial.printf("%7.3f %7.2f %7.2f %7.2f %7d ", vAcs, corrienteFiltradaA, potenciaFiltradaW, vBateria, escMicrosegundos);
  if (presionDisponible) {
    presion.read();
    float presionMbar = presion.pressure();
    float temperaturaC = presion.temperature();
    lecturaPresionValida = isfinite(presionMbar) && isfinite(temperaturaC) && presionMbar > 100.0f && presionMbar < 12000.0f && temperaturaC > -40.0f && temperaturaC < 85.0f;
    float profundidad = superficieCalibrada ? (presionMbar - presionSuperficieMbar) * 100.0f / (DENSIDAD_AGUA_KG_M3 * GRAVEDAD_M_S2) : 0;
    if (profundidad < 0) profundidad = 0;
    Serial.printf("%9.2f %7.2f %7.2f ", presionMbar, temperaturaC, profundidad);
  } else {
    lecturaPresionValida = false;
    Serial.print("       --      --      -- ");
  }
  if (imuDisponible && lecturaImuValida) {
    Serial.printf("%6.2f %6.2f %6.2f %7.1f %7.1f %7.1f ", aceleracionX, aceleracionY, aceleracionZ, giroX, giroY, giroZ);
    if (magnetometroDisponible && lecturaMagValida) Serial.printf("%7.1f %7.1f %7.1f ", magnetometroX, magnetometroY, magnetometroZ);
    else Serial.print("     --      --      -- ");
    Serial.printf("%7.1f %7.1f ", rollImu, pitchImu);
    if (magnetometroDisponible && lecturaMagValida) Serial.printf("%7.1f\n", rumboImu);
    else Serial.println("     --");
  } else Serial.print("    --     --     --      --      --      --      --      --      --      --      --      -- ");
  if (imuDisponible && lecturaImuValida) {
    Serial.printf("%4u %4u %4u %4u\n", calibracionSistema, calibracionGiroscopio,
                  calibracionAcelerometro, calibracionMagnetometro);
  } else Serial.println("  --   --   --   --");
  ++filasTabla;
}

void imprimirEncabezadoTabla() {
  Serial.println("\n ACS[V]   I[A]    P[W]  Bat[V] ESC[us] Press[mbar] Temp[C] Depth[m]   Ax[g]   Ay[g]   Az[g] Gx[dps] Gy[dps] Gz[dps] Mx[uT] My[uT] Mz[uT] Roll[deg] Pitch[deg] Yaw[deg] CalSys CalG CalA CalM");
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

bool hayFallaSensores() {
  return !escPwmDisponible ||
         !presionDisponible || !lecturaPresionValida ||
         !imuDisponible || !lecturaImuValida || fallosLecturaImu >= 5 ||
         !magnetometroDisponible || !lecturaMagValida || fallosLecturaMag >= 5 ||
         !lecturaAcsValida || !lecturaBateriaValida;
}

void actualizarLeds() {
  unsigned long ahora = millis();
  ModoLed modoDeseado;

  // En pausa se muestra LISTO aunque haya sensores ausentes; bateria baja conserva su aviso.
  if (ahora - inicioSistemaMs < TIEMPO_CONFIGURACION_LED_MS) modoDeseado = LED_CONFIG;
  else if (!sistemaOperando && voltajeBateriaActual < VOLTAJE_BATERIA_BAJO_V) modoDeseado = LED_BATERIA;
  else if (!sistemaOperando) modoDeseado = LED_LISTO;
  else if (hayFallaSensores()) modoDeseado = LED_ERROR;
  else if (voltajeBateriaActual < VOLTAJE_BATERIA_BAJO_V) modoDeseado = LED_BATERIA;
  else modoDeseado = LED_OPERANDO;

  if (modoDeseado != modoLed) {
    modoLed = modoDeseado;
    ultimoLedMs = ahora;
    estadoParpadeo = true;
    pixelConfiguracion = 0;
    pixels.setBrightness(40);

    if (modoLed == LED_CONFIG) {
      pixels.clear();
      pixels.setPixelColor(pixelConfiguracion++, pixels.Color(255, 120, 0));
      pixels.show();
    } else if (modoLed == LED_ERROR) {
      colorTodos(255, 0, 0);
    } else if (modoLed == LED_LISTO) {
      colorTodos(0, 120, 255);
    } else if (modoLed == LED_OPERANDO) {
      brilloOperacion = 5;
      pasoBrillo = 1;
      pixels.setBrightness(brilloOperacion);
      colorTodos(0, 0, 255);
    } else if (modoLed == LED_BATERIA) {
      colorTodos(255, 80, 0);
    } else {
      pixels.clear();
      pixels.show();
    }
  }

  if (modoLed == LED_ERROR && ahora - ultimoLedMs >= 400) {
    ultimoLedMs = ahora;
    estadoParpadeo = !estadoParpadeo;
    if (estadoParpadeo) colorTodos(255, 0, 0);
    else { pixels.clear(); pixels.show(); }
  } else if (modoLed == LED_CONFIG && ahora - ultimoLedMs >= 80) {
    ultimoLedMs = ahora;
    pixels.clear();
    pixels.setPixelColor(pixelConfiguracion++ % NUM_PIXELS, pixels.Color(255, 120, 0));
    pixels.show();
  } else if (modoLed == LED_OPERANDO && ahora - ultimoLedMs >= 20) {
    ultimoLedMs = ahora;
    if (brilloOperacion >= 40) pasoBrillo = -1;
    else if (brilloOperacion <= 5) pasoBrillo = 1;
    brilloOperacion += pasoBrillo;
    pixels.setBrightness(brilloOperacion);
    colorTodos(0, 0, 255);
  }
}

void colorTodos(uint8_t r, uint8_t g, uint8_t b) { for (uint16_t i = 0; i < NUM_PIXELS; ++i) pixels.setPixelColor(i, pixels.Color(r, g, b)); pixels.show(); }

void imprimirAyuda() {
  Serial.println("Comandos: ayuda | estado | stop | reiniciar");
  Serial.println("Agua: adelante/a, derecha/d, izquierda/i, roll_derecha/rd, roll_izquierda/ri, agua_off");
  Serial.println("Aire: aire_on, aire_off, aire1_on/off, aire2_on/off | valvula_on/off");
  Serial.println("Pinza: abrir, cerrar, servo 0..180 | Propulsor: esc 1000..2000, esc_off");
  Serial.println("Calibracion automatica: ACS712 al arrancar/en neutro; bateria guardada en memoria.");
  Serial.println("Calibracion: calibrar_bateria <V multimetro>, calibrar_superficie, calibrar_imu, calibrar_brujula");
  Serial.println("BNO055: calibracion automatica; gire el submarino en forma de ocho para mejorar el magnetometro/rumbo.");
  Serial.println("Control: iniciar | stop (Start alterna OPERANDO/LISTO desde el mando). Aro LED automatico.");
}
