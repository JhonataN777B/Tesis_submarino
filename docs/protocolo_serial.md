# Protocolo serial

## Parámetros

- Velocidad: 115200 baudios.
- Terminación de línea: `LF` (`\n`).
- Codificación recomendada: UTF-8.

## Firmware integrado actual

El firmware principal interpreta comandos de texto en minúsculas. Ejemplos:

```text
estado
adelante
aire_on
abrir
servo 120
esc 1300
stop
```

La respuesta de `estado` es texto legible con voltaje/corriente, MS5837 y MPU6050. Consulte `imprimirAyuda()` en el sketch para el conjunto completo.

## Pendiente de integración

La aplicación `interface/hud_superficie.py` conserva un protocolo anterior: transmite mensajes `$CTR,<id>` y espera telemetría `$MPU,...`. Ese formato no coincide con el firmware integrado actual. No operar con la interfaz hasta definir un único formato y actualizar ambos extremos.
