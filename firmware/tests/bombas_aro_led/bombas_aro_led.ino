#include <Adafruit_NeoPixel.h>

//=====================================================
// ARO LED
//=====================================================

#define LED_PIN 19
#define NUMPIXELS 16
#define BRILLO 40

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

//=====================================================
// PINES MOSFET LR7843
//=====================================================

const int MOTOR_ADELANTE = 4;

const int MOTOR_DER_1 = 9;
const int MOTOR_DER_2 = 11;

const int MOTOR_IZQ_1 = 13;
const int MOTOR_IZQ_2 = 12;

//=====================================================

String comando;

//=====================================================

void setup() {

  Serial.begin(115200);

  //------------ Pines de salida ------------

  pinMode(MOTOR_ADELANTE, OUTPUT);

  pinMode(MOTOR_DER_1, OUTPUT);
  pinMode(MOTOR_DER_2, OUTPUT);

  pinMode(MOTOR_IZQ_1, OUTPUT);
  pinMode(MOTOR_IZQ_2, OUTPUT);

  detenerMotores();

  //------------ LED ------------

  pixels.begin();
  pixels.setBrightness(BRILLO);

  apagar();

  Serial.println("--------------------------------");
  Serial.println(" ROV LISTO ");
  Serial.println("--------------------------------");
  Serial.println("Movimiento:");
  Serial.println("a");
  Serial.println("der");
  Serial.println("izq");
  Serial.println("stop");
  Serial.println("");
  Serial.println("LED:");
  Serial.println("error");
  Serial.println("config");
  Serial.println("listo");
  Serial.println("operando");
  Serial.println("bateria");
}

//=====================================================

void loop() {

  if (Serial.available()) {

    comando = Serial.readStringUntil('\n');

    comando.trim();

    //---------------- MOVIMIENTO ----------------

    if (comando == "a") {

      Serial.println("Adelante");

      detenerMotores();

      digitalWrite(MOTOR_ADELANTE, HIGH);
    }

    else if (comando == "der") {

      Serial.println("Derecha");

      detenerMotores();

      digitalWrite(MOTOR_DER_1, HIGH);
      digitalWrite(MOTOR_DER_2, HIGH);
    }

    else if (comando == "izq") {

      Serial.println("Izquierda");

      detenerMotores();

      digitalWrite(MOTOR_IZQ_1, HIGH);
      digitalWrite(MOTOR_IZQ_2, HIGH);
    }

    else if (comando == "stop") {

      Serial.println("Motores detenidos");

      detenerMotores();
    }

    //---------------- LED ----------------

    else if (comando == "error") {

      Serial.println("Modo ERROR");

      while (true) {

        if (Serial.available()) break;

        colorTodos(255,0,0);

        delay(400);

        apagar();

        delay(400);
      }
    }

    else if (comando == "config") {

      Serial.println("Modo CONFIG");

      while (true) {

        if (Serial.available()) break;

        for(int i=0;i<NUMPIXELS;i++){

          apagar();

          pixels.setPixelColor(i,pixels.Color(255,120,0));

          pixels.show();

          delay(80);

          if (Serial.available()) break;
        }
      }
    }

    else if(comando=="listo"){

      colorTodos(0,120,255);

      Serial.println("Listo");
    }

    else if(comando=="operando"){

      Serial.println("Operando");

      while(true){

        if(Serial.available()) break;

        for(int b=5;b<40;b++){

          pixels.setBrightness(b);

          colorTodos(0,0,255);

          delay(20);

          if(Serial.available()) break;
        }

        for(int b=40;b>5;b--){

          pixels.setBrightness(b);

          colorTodos(0,0,255);

          delay(20);

          if(Serial.available()) break;
        }

      }

      pixels.setBrightness(BRILLO);

    }

    else if(comando=="bateria"){

      Serial.println("Bateria baja");

      colorTodos(255,80,0);

    }

    else{

      Serial.println("Comando no reconocido");

    }

  }

}

//=====================================================
// FUNCIONES MOTORES
//=====================================================

void detenerMotores(){

  digitalWrite(MOTOR_ADELANTE,LOW);

  digitalWrite(MOTOR_DER_1,LOW);
  digitalWrite(MOTOR_DER_2,LOW);

  digitalWrite(MOTOR_IZQ_1,LOW);
  digitalWrite(MOTOR_IZQ_2,LOW);

}

//=====================================================
// FUNCIONES LED
//=====================================================

void colorTodos(int r,int g,int b){

  for(int i=0;i<NUMPIXELS;i++){

    pixels.setPixelColor(i,pixels.Color(r,g,b));

  }

  pixels.show();

}

void apagar(){

  pixels.clear();

  pixels.show();

}