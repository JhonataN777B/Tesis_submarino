# ROV / Submarino de tesis

Repositorio de trabajo para el ROV de tesis: firmware del ESP32-S3, aplicación de superficie y documentación técnica.

## Estructura

- `firmware/tests`: sketches independientes para validar sensores y actuadores.
- `firmware/integrated`: firmware principal `Control_Submarino_Unificado.ino`.
- `interface`: aplicación Python de superficie (HUD) y diagnóstico del mando.
- `hardware`: BOM, Gerbers, mapa de pines y referencias de PCB.
- `data`: muestras ligeras de telemetría; las grabaciones no se versionan.
- `docs`: protocolo, procedimiento de pruebas y notas técnicas.`n- `docs/evidencia`: fotos históricas de la primera versión construida, evidencia de PCB y enlace a pruebas FEA.

## Inicio rápido

1. Abra `firmware/integrated/Control_Submarino_Unificado.ino` en Arduino IDE.
2. Seleccione la placa ESP32-S3 y configure el puerto correcto.
3. Instale las librerías indicadas en el encabezado del sketch.
4. Para la interfaz, instale las dependencias de `interface/requirements.txt` y ajuste `PUERTO_COM` y la cámara en `hud_superficie.py`.

Antes de hacer una prueba con motores, mantenga el vehículo asegurado y ejecute primero `stop`.

