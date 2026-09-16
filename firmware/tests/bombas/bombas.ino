//
// CONTROL 4 BOMBAS SUBMARINO
// usando módulos MOSFET LR7843
//

// =====================================
// PINES ESP32
// =====================================

#define AI 18   // Arriba Izquierda
#define AD 19   // Arriba Derecha
#define BI 21   // Abajo Izquierda
#define BD 22   // Abajo Derecha

void setup() {

  Serial.begin(115200);

  pinMode(AI, OUTPUT);
  pinMode(AD, OUTPUT);
  pinMode(BI, OUTPUT);
  pinMode(BD, OUTPUT);

  apagarTodo();

  Serial.println("Sistema submarino listo");
  Serial.println("");
  Serial.println("Comandos:");
  Serial.println("derecha");
  Serial.println("izquierda");
  Serial.println("roll_derecha");
  Serial.println("roll_izquierda");
  Serial.println("stop");
}

void loop() {

  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');

    cmd.trim();

    // apagar todo antes de nuevo movimiento
    apagarTodo();

    // =====================================
    // GIRAR DERECHA
    // activa bombas lado izquierdo
    // =====================================
    if (cmd == "derecha") {

      Serial.println("Girando derecha");

      digitalWrite(AI, HIGH);
      digitalWrite(BI, HIGH);
    }

    // =====================================
    // GIRAR IZQUIERDA
    // activa bombas lado derecho
    // =====================================
    else if (cmd == "izquierda") {

      Serial.println("Girando izquierda");

      digitalWrite(AD, HIGH);
      digitalWrite(BD, HIGH);
    }

    // =====================================
    // ROLL DERECHA
    // diagonal:
    // arriba derecha + abajo izquierda
    // =====================================
    else if (cmd == "roll_derecha") {

      Serial.println("Roll derecha");

      digitalWrite(AD, HIGH);
      digitalWrite(BI, HIGH);
    }

    // =====================================
    // ROLL IZQUIERDA
    // diagonal:
    // arriba izquierda + abajo derecha
    // =====================================
    else if (cmd == "roll_izquierda") {

      Serial.println("Roll izquierda");

      digitalWrite(AI, HIGH);
      digitalWrite(BD, HIGH);
    }

    // =====================================
    // STOP
    // =====================================
    else if (cmd == "stop") {

      Serial.println("STOP");

      apagarTodo();
    }

    // =====================================
    // COMANDO INVÁLIDO
    // =====================================
    else {

      Serial.println("Comando no valido");
    }
  }
}

// =====================================
// APAGAR TODO
// =====================================

void apagarTodo() {

  digitalWrite(AI, LOW);
  digitalWrite(AD, LOW);
  digitalWrite(BI, LOW);
  digitalWrite(BD, LOW);
}
