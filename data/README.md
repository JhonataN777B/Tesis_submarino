# Datos

`telemetria_ejemplo.csv` es una muestra ilustrativa (no resultados experimentales) con el esquema actual de 27 columnas: tres metadatos de sesión y 24 campos de telemetría.

## Registros experimentales archivados

`logs/` contiene 92 CSV (12.98 MB) organizados por campaña: `Helice_4_aspas_tradicional` (36 registros), `Helice_Toroidal_4_aspas` (36) y `Prueba_piloto` (20). Son registros de telemetría capturados por la aplicación de superficie; se conservan las carpetas originales para distinguir las pruebas.

La aplicación genera nuevos logs en `interface/Logs_Submarino` durante cada ejecución y crea copias de respaldo allí. Esa carpeta de ejecución sigue excluida de Git; los registros archivados de esta campaña están en `data/logs/`. Las grabaciones del HUD no se incluyen como logs CSV.