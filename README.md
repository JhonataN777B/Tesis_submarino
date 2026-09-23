# ROV / Submarino de tesis

Repositorio de trabajo para el ROV de tesis: firmware del ESP32-S3, aplicación de superficie y documentación técnica.

## Estructura

- `firmware/tests`: sketches independientes para validar sensores y actuadores.
- `firmware/integrated`: firmware principal `Control_Submarino_Unificado.ino`.
- `interface`: HUD, diagnóstico del mando y scripts de control de superficie (ver `interface/README.md`).
- `hardware`: BOM, Gerbers, mapa de pines y referencias de PCB.
- `data`: muestras ligeras de telemetría; las grabaciones no se versionan.
- `docs`: protocolo, procedimiento de pruebas y notas técnicas.
- `docs/evidencia`: fotos históricas de la primera versión construida, renders de la versión 2, evidencia de PCB y enlace a pruebas FEA.

## Inicio rápido

1. Abra `firmware/integrated/Control_Submarino_Unificado.ino` en Arduino IDE.
2. Seleccione la placa ESP32-S3 y configure el puerto correcto.
3. Instale las librerías indicadas en el encabezado del sketch.
4. Para la interfaz, instale las dependencias de `interface/requirements.txt`; ajuste `PUERTO_COM` y la cámara según corresponda.
5. Para los scripts de control del mando, consulte `interface/README.md` y configure el puerto COM en `solo_control.py`.

Antes de hacer una prueba con motores, mantenga el vehículo asegurado y ejecute primero `stop`.
