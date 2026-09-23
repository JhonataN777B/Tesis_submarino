# Scripts de control de superficie

Estos scripts Python forman parte del trabajo de control con mando Bluetooth y ESP32. Se copiaron desde `Proyecto/Control_Submarino_Unificado`.

- [`prueba_control_mando.py`](prueba_control_mando.py): detecta el mando y muestra los índices y valores de ejes, botones y cruceta. Se usa para verificar el mapeo del mando.
- [`solo_control.py`](solo_control.py): envía comandos de movimiento al ESP32 por puerto serie usando el mando. Configura `PUERTO_COM` en el archivo según el puerto asignado al ESP32; usa 115200 baudios.

Instala las dependencias de [`requirements.txt`](requirements.txt). Antes de energizar actuadores, prueba con el vehículo asegurado y confirma el comando `stop`.
