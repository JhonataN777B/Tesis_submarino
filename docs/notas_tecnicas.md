# Notas técnicas

- El firmware integrado declara dependencias: ESP32Servo, Adafruit NeoPixel, MS5837, Adafruit MPU6050 y Adafruit Unified Sensor.
- El ESC inicia a 1000 µs por seguridad.
- El aro LED está configurado para 16 píxeles; ajustar `NUM_PIXELS` si el hardware cambia.
- La interfaz Python requiere cámara, control compatible con Pygame y ajuste manual de `PUERTO_COM`.
- Hay documentación de planificación y recomendaciones en `reference_documents/`.

## Material fuera del repositorio

Los archivos de Inventor, Blender, STL, videos, historial de versiones y simulaciones se dejaron en la carpeta original `Proyecto` para evitar subir activos pesados o derivados. Si se requiere versionarlos, usar Git LFS o un repositorio de diseño separado.
