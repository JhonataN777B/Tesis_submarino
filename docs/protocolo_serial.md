# Protocolo serial

## Parametros

- Velocidad: 115200 baudios.
- Fin de linea de comandos: LF (\n).
- El mismo puerto se usa para comandos y telemetria; cierre el Monitor Serial antes de conectar la interfaz Python.

## Telemetria del firmware integrado

El firmware transmite una fila cada 100 ms (10 Hz), con campos separados por espacios. Imprime una fila de titulos cada 20 muestras. Los sensores no disponibles se representan con --.

Orden de las 24 columnas actuales:

1. ACS712 ADC (V)
2. Corriente filtrada (A)
3. Potencia electrica (W)
4. Bateria (V)
5. Pulso ESC (us)
6. Presion (mbar)
7. Temperatura (C)
8. Profundidad (m)
9-11. Aceleracion X/Y/Z (g)
12-14. Giroscopio X/Y/Z (deg/s)
15-17. Magnetometro X/Y/Z (uT)
18. Roll (deg)
19. Pitch (deg)
20. Yaw/heading (deg)
21-24. Calibracion BNO055: sistema, giroscopio, acelerometro y magnetometro (0-3)

La interfaz acepta tanto las 24 columnas actuales como las 20 columnas de versiones anteriores; en el formato antiguo, los cuatro estados de calibracion quedan vacios. Los datos recibidos se presentan en el HUD y se guardan como CSV con una columna por campo, coma como separador y punto decimal. Cada sesion tiene una copia espejo en Respaldo.

## Comandos

- Estado y seguridad: ayuda, estado, iniciar, stop.
- Reinicio: reiniciar (tambien acepta reinicio o reset); detiene las salidas y reinicia el ESP32.
- ESC reversible: esc 1000..2000; neutro 1500 us; esc_off vuelve a neutro.
- Bombas de movimiento: adelante/a, derecha/d, izquierda/i, roll_derecha/rd, roll_izquierda/ri, agua_off.
- Aire: aire_on, aire_off, aire1_on, aire1_off, aire2_on, aire2_off.
- Electrovalvula: valvula_on, valvula_off.
- Pinza: abrir, cerrar, servo 0..180.
- Calibracion de bateria: calibrar_bateria <voltaje del multimetro>; guarda el factor en memoria no volatil del ESP32.
- Presion: calibrar_superficie con el sensor fuera del agua.
- BNO055: calibrar_imu y calibrar_brujula muestran estado de calibracion; para mejorar el magnetometro, mueva el vehiculo lentamente en forma de ocho lejos de metal y corrientes del motor.

## Interfaz de superficie

interface/solo_control.py lee continuamente el flujo serial, envia comandos del mando y registra una sesion. El archivo interface/prueba_control_mando.py solo diagnostica los indices del mando; no envia comandos al submarino.
