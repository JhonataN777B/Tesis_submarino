# Notas tecnicas

## Firmware ESP32-S3

- Librerias externas: ESP32Servo, Adafruit NeoPixel, MS5837 compatible con MS5837.h y DFRobot_BNO055. Wire, math y Preferences pertenecen al core/SDK.
- BNO055/SEN0374 por I2C: SDA GPIO 8, SCL GPIO 9; direcciones probadas 0x28 y 0x29. Se selecciona modo NDOF y se leen aceleracion, giroscopio, magnetometro y Euler (head/roll/pitch).
- El rumbo se normaliza a 0-360 grados; la calibracion/perturbacion magnetica afecta su confiabilidad.
- El MS5837, ACS712 y divisor de bateria alimentan la telemetria. El factor de bateria se calibra con calibrar_bateria <V_multimetro> y se conserva en Preferences.
- La telemetria integrada sale cada 100 ms (10 Hz). Los valores de sensores ausentes se escriben como --.
- El ESC reversible usa 1000-2000 us y neutro 1500 us. GPIO15 controla la señal a traves del NPN previsto por el montaje.
- El aro NeoPixel usa GPIO21 y 16 pixeles; el umbral configurado de bateria baja es 10.5 V.

## Interfaz y archivos de sesion

- La aplicacion actual es interface/solo_control.py; interface/prueba_control_mando.py sirve solo para diagnosticar el PG-9076.
- El HUD requiere OpenCV, NumPy, PySerial y Pygame. El indice de camara y COM se ajustan en el script.
- La interfaz guarda CSV con coma entre columnas y punto decimal, y mantiene una copia en Respaldo. Los CSV y videos de usuario en carpetas de ejecucion se excluyen del repositorio.
- Los videos seleccionados de prueba se conservan bajo docs/evidencia/operacion.

## Activos de hardware

- hardware contiene el mapa de pines, BOM/Gerbers y STL elegidos para imprimir.
- Los originales/editables de Inventor, Blender y simulacion permanecen en las carpetas de trabajo originales.
