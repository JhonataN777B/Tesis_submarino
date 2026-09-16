# Procedimiento de pruebas

1. Inspección: comprobar conectores, polaridad, fusibles y aislamiento.
2. Sin cargas: cargar el sketch y abrir el monitor serie a 115200.
3. Sensores: ejecutar `estado`; registrar detección de MPU6050 y MS5837.
4. Actuadores: probar un sketch de `firmware/tests` por vez, con alimentación limitada.
5. Integración: fijar físicamente el ROV, iniciar el firmware principal y validar `stop` antes de cualquier movimiento.
6. Superficie: integrar protocolo serial y probar mando/cámara sin propulsión antes de inmersión.

Registre fecha, versión de firmware, hardware usado, resultado y anomalías en una bitácora externa o en un archivo ligero dentro de `data/`.
