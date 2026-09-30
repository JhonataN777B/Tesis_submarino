# Control de superficie

## Aplicacion integrada

solo_control.py es la aplicacion activa para el mando Bluetooth, ESP32, HUD de camara, visualizacion de telemetria, graficas, logs y grabacion del HUD.

Instale las dependencias con:

    pip install -r requirements.txt

Ajuste PUERTO_COM y CAMARA dentro de solo_control.py. El programa usa 115200 baudios y comparte el puerto serie con los comandos del ESP32. Cierre el Monitor Serial de Arduino antes de ejecutar Python.

El firmware emite telemetria a 10 Hz. El panel muestra la lectura de sensores; los valores ausentes aparecen como --. Los logs CSV de cada sesion y su espejo de respaldo se escriben en interface/Logs_Submarino cuando el programa se ejecuta desde esta carpeta. Las grabaciones iniciadas desde el mando se guardan en Grabaciones_HUD.

## Mando PG-9076

Indices confirmados con Pygame:

| Control | Funcion |
| --- | --- |
| Palanca izquierda vertical | Control del ESC (1000-2000 us; neutro 1500 us) |
| Palanca derecha horizontal | Orden de giro derecha/izquierda |
| Palanca derecha vertical | Orden de roll derecha/izquierda |
| A, boton 0 | Solicita reinicio seguro del ESP32 |
| B, boton 1 | Alterna abrir/cerrar pinza |
| X, boton 3 | Alterna bomba de aire 1 |
| Y, boton 4 | Alterna bomba de aire 2 |
| Boton 7 | Alterna electrovalvula |
| Boton 10 | Inicia/detiene grabacion del HUD |
| Start, boton 11 | Alterna LISTO/OPERANDO |

Start deja el vehiculo en LISTO al detener y exige centrar las palancas antes de habilitar movimiento al volver a OPERANDO. El boton A solicita el reinicio seguro del ESP32.

La tecla G cambia entre HUD y graficas; R alterna la grabacion y Q cierra la ventana. El boton 10 del mando tambien alterna la grabacion.

## Diagnostico independiente del mando

prueba_control_mando.py muestra ejes, botones y cruceta sin controlar el ROV. Uselo para comprobar un mando o volver a confirmar su mapeo antes de operar.
