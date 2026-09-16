# Mapa de pines — ESP32-S3

Fuente: `firmware/integrated/Control_Submarino_Unificado.ino`. Validar en el PCB antes de conectar cargas.

| GPIO | Señal | Función |
| --- | --- | --- |
| 1 | ACS712 | Corriente (ADC) |
| 2 | SERVO_PINZA | Servo de pinza |
| 4 | BOMBA_AI | Bomba de agua, adelante izquierda |
| 5 | BOMBA_AD | Bomba de agua, adelante derecha |
| 6 | AIRE_1 | Bomba de aire 1 |
| 7 | AIRE_2 | Bomba de aire 2 |
| 8 | I2C_SDA | Bus I²C: MPU6050 y MS5837 |
| 9 | I2C_SCL | Bus I²C: MPU6050 y MS5837 |
| 10 | BATERIA | Voltaje de batería (ADC) |
| 11 | BOMBA_BI | Bomba de agua, atrás izquierda |
| 12 | BOMBA_BD | Bomba de agua, atrás derecha |
| 13 | ELECTROVALVULA | Electroválvula de lastre |
| 15 | ESC | Señal PWM del propulsor |
| 21 | NEOPIXEL | Aro LED (16 píxeles configurados) |

Los GPIO 1 y 10 usan ADC de 12 bits. El firmware presupone divisor de batería de 30 kΩ/10 kΩ y divisor 10 kΩ/10 kΩ para la salida del ACS712.
