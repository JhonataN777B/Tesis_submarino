#include <Adafruit_NeoPixel.h>

#define PIN 4
#define NUMPIXELS 16

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// ===============================
// BRILLO BAJO PARA USB / VIN
// ===============================
#define BRILLO 40

void setup() {

  Serial.begin(115200);

  pixels.begin();
  pixels.setBrightness(BRILLO);

  pixels.clear();
  pixels.show();

  Serial.println("Sistema LED listo");
  Serial.println("");
  Serial.println("Comandos:");
  Serial.println("error");
  Serial.println("config");
  Serial.println("listo");
  Serial.println("operando");
  Serial.println("bateria");
}

void loop() {

  if (Serial.available()) {

    String comando = Serial.readStringUntil('\n');
    comando.trim();

    // ===============================
    // ERROR -> ROJO PARPADEANDO
    // ===============================
    if (comando == "error") {

      Serial.println("Modo ERROR");

      while (true) {

        if (Serial.available()) break;

        colorTodos(255, 0, 0);
        delay(400);

        apagar();
        delay(400);
      }
    }

    // ===============================
    // CONFIG -> AMARILLO GIRANDO
    // ===============================
    else if (comando == "config") {

      Serial.println("Modo CONFIGURANDO");

      while (true) {

        if (Serial.available()) break;

        for (int i = 0; i < NUMPIXELS; i++) {

          apagar();

          pixels.setPixelColor(i, pixels.Color(255, 120, 0));

          pixels.show();

          delay(80);

          if (Serial.available()) break;
        }
      }
    }

    // ===============================
    // LISTO -> CIAN FIJO
    // ===============================
    else if (comando == "listo") {

      Serial.println("Modo LISTO");

      colorTodos(0, 120, 255);
    }

    // ===============================
    // OPERANDO -> AZUL RESPIRANDO
    // ===============================
    else if (comando == "operando") {

      Serial.println("Modo OPERANDO");

      while (true) {

        if (Serial.available()) break;

        // subir brillo
        for (int b = 5; b < 40; b++) {

          pixels.setBrightness(b);
          colorTodos(0, 0, 255);

          delay(20);

          if (Serial.available()) break;
        }

        // bajar brillo
        for (int b = 40; b > 5; b--) {

          pixels.setBrightness(b);
          colorTodos(0, 0, 255);

          delay(20);

          if (Serial.available()) break;
        }
      }

      pixels.setBrightness(BRILLO);
    }

    // ===============================
    // BATERIA -> NARANJA
    // ===============================
    else if (comando == "bateria") {

      Serial.println("Bateria baja");

      colorTodos(255, 80, 0);
    }
  }
}

// ======================================
// FUNCIONES
// ======================================

void colorTodos(int r, int g, int b) {

  for (int i = 0; i < NUMPIXELS; i++) {

    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }

  pixels.show();
}

void apagar() {

  pixels.clear();
  pixels.show();
}