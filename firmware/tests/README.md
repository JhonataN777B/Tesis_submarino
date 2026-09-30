# Pruebas independientes de hardware

Cada carpeta contiene un sketch Arduino independiente. Conecte un elemento bajo prueba a la vez, verifique alimentacion/GND comun y confirme el pin en ../../hardware/pin_map.md antes de cargarlo.

| Carpeta | Elemento |
| --- | --- |
| aro_led | Aro NeoPixel |
| blink_led | LED basico |
| bombas | Bombas de agua |
| bombas_aro_led | Prueba combinada de bombas y aro LED |
| garra_v1, garra_v2 | Servo de pinza |
| sensor_magnetico | Sensor magnetico |
| sensor_presion | Sensor de presion |

El diagnostico independiente del mando Bluetooth es un programa Python, no un sketch: interface/prueba_control_mando.py.
