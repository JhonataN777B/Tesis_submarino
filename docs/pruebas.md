# Procedimiento de pruebas

1. Inspeccion mecanica y electrica: revisar conectores, polaridad, aislamiento, sujecion del sensor y estanqueidad antes de instalar electronica sensible.
2. Prueba de sensores sin actuadores: cargar el firmware integrado, iniciar a 115200 baudios y confirmar la deteccion del MS5837 y el BNO055/SEN0374 en el monitor. Cerrar el monitor antes de ejecutar Python.
3. Orientacion: comprobar las tres rotaciones del vehiculo y la telemetria. Giro alrededor del eje longitudinal debe cambiar roll; morro arriba/abajo debe cambiar pitch; giro horizontal alrededor del eje vertical debe cambiar yaw/heading. Confirmar que el rumbo deja de aparecer como -- y revisar la calibracion del magnetometro.
4. Mando: ejecutar interface/prueba_control_mando.py para verificar indices de ejes y botones sin accionar el ROV.
5. Actuadores en banco: mantener el ROV asegurado, comprobar ESC en neutro (1500 us), y probar bombas, aire, electrovalvula y pinza individualmente. Usar stop si algo responde distinto a lo esperado.
6. Interfaz integrada: configurar COM y camara en interface/solo_control.py; verificar telemetria, HUD, mandos, estado LISTO/OPERANDO y reinicio seguro.
7. Captura: iniciar/detener video con boton 10 y confirmar que el contador aparezca en HUD. Revisar el CSV de la sesion y su copia en Logs_Submarino/Respaldo.
8. Prueba en agua: empezar con el vehiculo asegurado y una inmersion progresiva. Demostrar movimiento teleoperado en X/Y/Z/yaw, estabilidad razonable de roll/pitch, video y pinza. Registrar profundidad, condiciones, bateria, comandos y anomalias.

Los angulos se monitorizan y registran; este firmware no aplica estabilizacion automatica de pitch/roll ni control automatico de profundidad. Para cada prueba registre fecha, version de firmware, hardware, resultado y fallas.
