"""Controla el submarino con el mando PG-9076 y el ESP32 por puerto serie."""

import os
import sys
import time
import math
import ctypes
import csv
import shutil
from collections import deque
from datetime import datetime

# El programa usa una ventana oculta; permite que SDL actualice el mando sin foco.
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame
import serial

# La interfaz usa OpenCV como en "vsc interfaz.txt". Si falta alguna dependencia,
# el control del submarino sigue funcionando por consola.
try:
    import cv2
    import numpy as np
except ImportError as error:
    cv2 = None
    np = None
    print(f"Interfaz visual no disponible ({error}); el control seguira por consola.")


# Cambiar COM3 si Windows asigna otro puerto al ESP32.
PUERTO_COM = "COM3"
BAUDIOS = 115200
DEADZONE = 0.25
REENVIO_MOVIMIENTO_S = 0.15
ESPERA_ESP32_S = 30
TIEMPO_SIN_TELEMETRIA_S = 10.0
TIEMPO_SUAVIZADO_ACTITUD_S = 0.45

# Telemetria tabular enviada por Control_Submarino_Unificado.ino, en el orden
# de sus 24 columnas, incluidas las cuatro calibraciones internas del BNO055.
# El mismo puerto serie se comparte con los comandos.
CAMPOS_TELEMETRIA = (
    "acs_v", "corriente_a", "potencia_w", "bateria_v", "esc_us",
    "presion_mbar", "temperatura_c", "profundidad_m",
    "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
    "mx_ut", "my_ut", "mz_ut", "roll_deg", "pitch_deg", "yaw_deg",
    "cal_sistema", "cal_giroscopio", "cal_acelerometro", "cal_magnetometro",
)
COLUMNAS_LOG = (
    "acs712_adc_v", "corriente_motor_a", "potencia_motor_w", "bateria_v", "esc_us",
    "presion_mbar", "temperatura_c", "profundidad_m", "aceleracion_x_g", "aceleracion_y_g",
    "aceleracion_z_g", "giroscopio_x_dps", "giroscopio_y_dps", "giroscopio_z_dps",
    "magnetometro_x_ut", "magnetometro_y_ut", "magnetometro_z_ut", "roll_deg", "pitch_deg", "yaw_deg",
    "bno055_calibracion_sistema_0_3", "bno055_calibracion_giroscopio_0_3",
    "bno055_calibracion_acelerometro_0_3", "bno055_calibracion_magnetometro_0_3",
)
telemetria = {campo: None for campo in CAMPOS_TELEMETRIA}
ultima_telemetria = 0.0
fallo_reportado_esp32 = False
historial_telemetria = deque(maxlen=600)  # Ultimos 60 s a 10 Hz.
escritores_log = []
salidas_log_pendientes = []
inicio_logs_monotonic = None
logs_activos = False
respaldo_log_activo = False
ruta_log_respaldo = None
ultimo_intento_reparacion_log = 0.0
GRUPOS_SENSORES = {
    "ACS712": ("acs_v",),
    "BATERIA": ("bateria_v",),
    "MS5837": ("presion_mbar", "temperatura_c", "profundidad_m"),
    "BNO055": (
        "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
        "mx_ut", "my_ut", "mz_ut", "roll_deg", "pitch_deg", "yaw_deg",
    ),
}
roll_hud_filtrado = 0.0
pitch_hud_filtrado = 0.0
ultimo_filtro_actitud = 0.0

# Opciones visuales heredadas del HUD de "vsc interfaz.txt".
CAMARA = 1  # La camara principal suele ser 0; la webcam USB secundaria, 1.
VAL_CONTRASTE = 1.24
VAL_SATURACION = 1.70
CARPETA_VIDEOS = "Grabaciones_HUD"
CARPETA_LOGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Logs_Submarino")
NOMBRE_VENTANA = "Sistema de Vision Submarino"
try:
    ANCHO_PANTALLA = ctypes.windll.user32.GetSystemMetrics(0)
    ALTO_PANTALLA = ctypes.windll.user32.GetSystemMetrics(1)
except (AttributeError, OSError):
    ANCHO_PANTALLA, ALTO_PANTALLA = 1280, 720

captura = None
grabando = False
video_writer = None
inicio_grabacion = None
INTERFAZ_ACTIVA = cv2 is not None
vista_graficas = False

# Mapeo medido en prueba control.py; Pygame entrega algunos ejes duplicados.
EJE_IZQUIERDO_Y = 1
# EJE[2] duplica al motor; EJE[5] duplica el vertical derecho.
EJE_DERECHO_X = 3
EJE_DERECHO_Y = 4

# Indices confirmados en la prueba de Windows/Pygame.
BOTON_A, BOTON_B, BOTON_X, BOTON_Y = 0, 1, 3, 4
BOTON_START = 11
BOTON_GRABAR = 10
BOTON_REINICIAR_ESP32 = BOTON_A
BOTON_VALVULA = 7

# Debe coincidir con ESC_REVERSIBLE en el sketch Arduino.
ESC_REVERSIBLE = True
PULSO_ESC_NEUTRO = 1500 if ESC_REVERSIBLE else 1000


def limitar(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


def actualizar_telemetria(linea):
    """Acepta filas actuales (24 datos) y antiguas (20 datos, sin cal BNO055)."""
    global ultima_telemetria, fallo_reportado_esp32
    texto = linea.strip()
    if "ERROR:" in texto.upper():
        fallo_reportado_esp32 = True
    if texto.startswith("ESP32:"):
        texto = texto[len("ESP32:"):].strip()
    columnas = texto.split()
    # El firmware previo enviaba 20 columnas; el que integra BNO055 agrega
    # cuatro estados de calibracion. Rechaza encabezados y mensajes parciales.
    if len(columnas) not in (20, len(CAMPOS_TELEMETRIA)):
        return False
    # Los campos nuevos quedan vacios al recibir una fila del firmware antiguo.
    nuevos = {campo: None for campo in CAMPOS_TELEMETRIA}
    try:
        for nombre, token in zip(CAMPOS_TELEMETRIA, columnas[:len(CAMPOS_TELEMETRIA)]):
            nuevos[nombre] = None if token == "--" else float(token.replace(",", "."))
    except ValueError:
        return False
    telemetria.update(nuevos)
    ahora = time.monotonic()
    ultima_telemetria = ahora
    historial_telemetria.append((ahora, nuevos.copy()))
    registrar_fila_log(nuevos, ahora)
    return True


def iniciar_logs():
    """Abre CSV de sesion y una copia espejo en la carpeta de respaldo."""
    global inicio_logs_monotonic, logs_activos, respaldo_log_activo, ruta_log_respaldo
    carpeta_respaldo = os.path.join(CARPETA_LOGS, "Respaldo")
    try:
        os.makedirs(CARPETA_LOGS, exist_ok=True)
        os.makedirs(carpeta_respaldo, exist_ok=True)
    except OSError as error:
        print(f"No se pudo crear la carpeta de logs: {error}")
        return False
    marca = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    nombre = f"telemetria_{marca}.csv"
    encabezado = ["fecha_hora_local", "tiempo_s", "modo"] + list(COLUMNAS_LOG)
    ruta_log_respaldo = os.path.join(carpeta_respaldo, nombre)

    for carpeta, etiqueta in ((CARPETA_LOGS, "principal"), (carpeta_respaldo, "respaldo")):
        ruta = os.path.join(carpeta, nombre)
        try:
            archivo = open(ruta, "w", newline="", encoding="utf-8-sig")
            # CSV estandar: coma entre columnas y punto decimal, para que Excel
            # no confunda las comas decimales con separadores de campos.
            escritor = csv.writer(archivo, delimiter=",")
            escritor.writerow(encabezado)
            archivo.flush()
            escritores_log.append((archivo, escritor, ruta, etiqueta))
            print(f"Log {etiqueta}: {ruta}")
        except OSError as error:
            print(f"No se pudo abrir el log {etiqueta}: {error}")

    inicio_logs_monotonic = time.monotonic()
    logs_activos = bool(escritores_log)
    respaldo_log_activo = any(etiqueta == "respaldo" for _, _, _, etiqueta in escritores_log)
    if logs_activos:
        print("Captura CSV activa." if not respaldo_log_activo else
              "Captura CSV activa con copia espejo de respaldo.")
    return logs_activos


def registrar_fila_log(valores, ahora):
    """Escribe y vacia cada muestra para conservar los datos ante un cierre abrupto."""
    global logs_activos, respaldo_log_activo
    if not logs_activos or inicio_logs_monotonic is None:
        return
    marca_hora = datetime.now().astimezone().isoformat(timespec="milliseconds")
    tiempo_s = f"{ahora - inicio_logs_monotonic:.3f}"
    modo = "OPERANDO" if operando else "LISTO"
    fila = [marca_hora, tiempo_s, modo]
    fila.extend(
        "" if valores.get(campo) is None else str(valores[campo])
        for campo in CAMPOS_TELEMETRIA
    )

    recuperar_log_principal(ahora)
    for salida in list(escritores_log):
        archivo, escritor, ruta, etiqueta = salida
        try:
            escritor.writerow(fila)
            archivo.flush()
        except OSError as error:
            try:
                archivo.close()
            except OSError:
                pass
            escritores_log.remove(salida)
            salidas_log_pendientes.append(salida[2:])
            if etiqueta == "respaldo":
                respaldo_log_activo = False
            print(f"Error escribiendo log {etiqueta} ({ruta}): {error}")
    logs_activos = bool(escritores_log)
    respaldo_log_activo = any(etiqueta == "respaldo" for _, _, _, etiqueta in escritores_log)


def recuperar_log_principal(ahora):
    """Restaura el CSV principal desde el espejo cuando vuelve a estar disponible."""
    global ultimo_intento_reparacion_log
    if not salidas_log_pendientes or ruta_log_respaldo is None:
        return
    if ahora - ultimo_intento_reparacion_log < 15.0:
        return
    salida_respaldo = next(
        (salida for salida in escritores_log if salida[3] == "respaldo"), None
    )
    if salida_respaldo is None:
        return

    ultimo_intento_reparacion_log = ahora
    try:
        salida_respaldo[0].flush()
        for ruta, etiqueta in list(salidas_log_pendientes):
            if etiqueta != "principal":
                continue
            shutil.copyfile(ruta_log_respaldo, ruta)
            archivo = open(ruta, "a", newline="", encoding="utf-8-sig")
            escritor = csv.writer(archivo, delimiter=",")
            escritores_log.append((archivo, escritor, ruta, etiqueta))
            salidas_log_pendientes.remove((ruta, etiqueta))
            print("Log principal recuperado y sincronizado desde Respaldo.")
    except OSError:
        # Si el archivo sigue abierto en otra aplicacion, el respaldo continua capturando.
        return


def cerrar_logs():
    global logs_activos, respaldo_log_activo
    for archivo, _, _, etiqueta in list(escritores_log):
        try:
            archivo.flush()
            archivo.close()
        except OSError as error:
            print(f"Error cerrando log {etiqueta}: {error}")
    escritores_log.clear()
    if ruta_log_respaldo is not None:
        for ruta, etiqueta in list(salidas_log_pendientes):
            if etiqueta == "principal" and os.path.exists(ruta_log_respaldo):
                try:
                    shutil.copyfile(ruta_log_respaldo, ruta)
                    print("Log principal actualizado desde Respaldo al cerrar la sesion.")
                    salidas_log_pendientes.remove((ruta, etiqueta))
                except OSError as error:
                    print(f"No se pudo sincronizar el log principal al cerrar: {error}")
    salidas_log_pendientes.clear()
    logs_activos = False
    respaldo_log_activo = False


def sensores_fallando():
    """Nombra grupos con campos ausentes; 0 A y 0 W no se consideran fallas."""
    fallas = []
    for nombre, campos in GRUPOS_SENSORES.items():
        if any(
            telemetria.get(campo) is None or not math.isfinite(telemetria[campo])
            for campo in campos
        ):
            fallas.append(nombre)
    if fallo_reportado_esp32:
        fallas.append("ESP32/ESC")
    return fallas


def texto_borde(imagen, texto, posicion, escala, color=(255, 255, 255), grosor=1):
    fuente = cv2.FONT_HERSHEY_SIMPLEX
    # Sin una segunda capa negra: el grosor del contorno desplazaba visualmente las letras.
    cv2.putText(imagen, texto, posicion, fuente, escala, color, grosor, cv2.LINE_AA)


def valor_hud(campo, formato=".2f", sufijo=""):
    valor = telemetria.get(campo)
    return "--" if valor is None else f"{valor:{formato}}{sufijo}"


def preparar_interfaz():
    """Inicia el HUD/camara sin abrir otro puerto serie."""
    global captura, INTERFAZ_ACTIVA
    if cv2 is None:
        INTERFAZ_ACTIVA = False
        return False
    try:
        captura = cv2.VideoCapture(CAMARA, cv2.CAP_DSHOW)
        captura.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        captura.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        captura.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not captura.isOpened():
            captura.release()
            captura = None
            print("Camara no disponible; el HUD se mostrara sobre fondo oscuro.")
        cv2.namedWindow(NOMBRE_VENTANA, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(NOMBRE_VENTANA, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        print("HUD activo: G alterna HUD/graficas; R o boton 10 graba video; Q cierra.")
        return True
    except Exception as error:
        if captura is not None:
            captura.release()
            captura = None
        INTERFAZ_ACTIVA = False
        print(f"No se pudo iniciar la ventana HUD: {error}")
        return False


def dibujar_hud(operando, ahora):
    """Dibuja la telemetria recibida en la interfaz inspirada en vsc interfaz."""
    global captura, roll_hud_filtrado, pitch_hud_filtrado, ultimo_filtro_actitud
    escala = max(0.8, min(1.25, ALTO_PANTALLA / 800.0))
    if captura is not None:
        ok, fotograma = captura.read()
    else:
        ok, fotograma = False, None
    if not ok or fotograma is None:
        imagen = np.zeros((ALTO_PANTALLA, ANCHO_PANTALLA, 3), dtype=np.uint8)
    else:
        imagen = cv2.resize(fotograma, (ANCHO_PANTALLA, ALTO_PANTALLA), interpolation=cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * VAL_SATURACION, 0, 255)
        imagen = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
        imagen = cv2.convertScaleAbs(imagen, alpha=VAL_CONTRASTE, beta=0)

    alto, ancho = imagen.shape[:2]
    # Velos translucidos para que las lecturas sean legibles con cualquier camara.
    capa = imagen.copy()
    cv2.rectangle(capa, (0, 0), (ancho, int(90 * escala)), (8, 16, 22), -1)
    cv2.rectangle(capa, (0, int(alto * 0.72)), (ancho, alto), (8, 16, 22), -1)
    cv2.addWeighted(capa, 0.68, imagen, 0.32, 0, imagen)

    margen = int(28 * escala)
    vivo = ultima_telemetria > 0 and ahora - ultima_telemetria <= TIEMPO_SIN_TELEMETRIA_S
    fallas_sensores = sensores_fallando() if vivo else []
    error_sensores = bool(fallas_sensores)
    bateria_baja = telemetria.get("bateria_v") is not None and telemetria["bateria_v"] < 10.5
    estado = "OPERANDO" if operando else "LISTO"
    color_modo = (0, 210, 255) if operando else (255, 220, 0)
    if not vivo:
        texto_salud, color_salud = ("SIN TELEMETRIA" if ultima_telemetria else "ESPERANDO TELEMETRIA"), (0, 150, 255)
    elif error_sensores:
        texto_salud, color_salud = f"FALLA: {', '.join(fallas_sensores)}", (0, 0, 255)
    elif bateria_baja:
        texto_salud, color_salud = "BATERIA BAJA", (0, 150, 255)
    else:
        texto_salud, color_salud = "SENSORES OK", (0, 220, 0)
    texto_borde(imagen, "SISTEMA DE VISION SUBMARINO", (margen, int(42 * escala)), 0.8 * escala, (0, 210, 255), 2)
    texto_borde(imagen, time.strftime("%d/%m/%Y  %H:%M:%S"), (int(ancho * 0.40), int(42 * escala)), 0.55 * escala)
    texto_borde(imagen, f"MODO: {estado}", (int(ancho * 0.70), int(38 * escala)), 0.52 * escala, color_modo, 2)
    texto_borde(imagen, texto_salud, (int(ancho * 0.70), int(75 * escala)), 0.48 * escala, color_salud, 2)
    texto_borde(imagen, f"VALVULA: {'ON' if valvula_activa else 'OFF'}",
                (margen, int(75 * escala)), 0.46 * escala,
                (80, 220, 120) if valvula_activa else (190, 200, 205), 1)
    if logs_activos:
        texto_borde(imagen, "LOG CSV + RESPALDO" if respaldo_log_activo else "LOG CSV ACTIVO",
                    (int(ancho * 0.40), int(75 * escala)), 0.46 * escala,
                    (80, 220, 120) if respaldo_log_activo else (0, 180, 255), 1)

    # El rumbo fusionado del BNO055 solo se muestra cuando llega una lectura valida.
    cx, cy, radio = int(125 * escala), int(170 * escala), int(72 * escala)
    cv2.circle(imagen, (cx, cy), radio, (220, 220, 220), max(1, int(2 * escala)), cv2.LINE_AA)
    cv2.circle(imagen, (cx, cy), int(radio * 0.55), (120, 120, 120), 1, cv2.LINE_AA)
    texto_borde(imagen, "N", (cx - int(7 * escala), cy - radio - int(8 * escala)), 0.45 * escala, (0, 255, 255))
    rumbo = telemetria.get("yaw_deg")
    if rumbo is not None:
        angulo = math.radians(rumbo - 90)
        punta = (int(cx + radio * math.cos(angulo)), int(cy + radio * math.sin(angulo)))
        cv2.line(imagen, (cx, cy), punta, (0, 255, 255), max(1, int(3 * escala)), cv2.LINE_AA)
        texto_borde(imagen, f"{rumbo:.1f} deg", (cx - int(45 * escala), cy + radio + int(25 * escala)), 0.48 * escala)
    else:
        texto_borde(imagen, "RUMBO --", (cx - int(48 * escala), cy + radio + int(25 * escala)), 0.46 * escala, (0, 190, 255))
    cal_sistema = valor_hud("cal_sistema", ".0f")
    cal_gyro = valor_hud("cal_giroscopio", ".0f")
    cal_accel = valor_hud("cal_acelerometro", ".0f")
    cal_mag = valor_hud("cal_magnetometro", ".0f")
    texto_borde(
        imagen,
        f"CAL BNO055  SYS {cal_sistema}  G {cal_gyro}  A {cal_accel}  M {cal_mag}",
        (margen, cy + radio + int(55 * escala)), 0.43 * escala,
        (180, 220, 230), 1,
    )

    # La linea queda anclada en la cruz central y gira segun pitch, como una balanza.
    hx, hy = ancho // 2, int(alto * 0.43)
    roll_objetivo = telemetria.get("roll_deg")
    pitch_objetivo = telemetria.get("pitch_deg")
    dt_filtro = 1.0 / 60.0 if ultimo_filtro_actitud == 0.0 else max(0.0, min(0.1, ahora - ultimo_filtro_actitud))
    ultimo_filtro_actitud = ahora
    alpha_actitud = 1.0 - math.exp(-dt_filtro / TIEMPO_SUAVIZADO_ACTITUD_S)
    if roll_objetivo is not None:
        diferencia_roll = (roll_objetivo - roll_hud_filtrado + 180.0) % 360.0 - 180.0
        roll_hud_filtrado += alpha_actitud * diferencia_roll
    if pitch_objetivo is not None:
        pitch_hud_filtrado += alpha_actitud * (pitch_objetivo - pitch_hud_filtrado)
    roll = roll_hud_filtrado
    pitch = pitch_hud_filtrado
    longitud = int(min(ancho * 0.20, 300 * escala))
    angulo_pitch = math.radians(pitch)
    dx = int(longitud * math.cos(angulo_pitch))
    dy = int(longitud * math.sin(angulo_pitch))
    cv2.line(imagen, (hx - dx, hy - dy), (hx + dx, hy + dy),
             (0, 0, 255), max(2, int(3 * escala)), cv2.LINE_AA)
    cv2.drawMarker(imagen, (hx, hy), (255, 255, 255), cv2.MARKER_CROSS, int(22 * escala), 2, cv2.LINE_AA)
    roll_texto = f"{roll:.1f} deg" if roll_objetivo is not None else "--"
    pitch_texto = f"{pitch:.1f} deg" if pitch_objetivo is not None else "--"
    texto_borde(imagen, f"ROLL {roll_texto}   PITCH {pitch_texto}",
                (hx - int(190 * escala), hy + int(95 * escala)), 0.48 * escala)

    # Bateria y ACS712: presenta el valor medido y resalta el umbral bajo ya definido.
    bateria = telemetria.get("bateria_v")
    color_bateria = (0, 0, 255) if bateria is not None and bateria < 10.5 else (0, 220, 0)
    texto_borde(imagen, f"BATERIA  {valor_hud('bateria_v', '.2f', ' V')}",
                (ancho - int(300 * escala), int(105 * escala)), 0.62 * escala, color_bateria, 2)

    # Indicador persistente y visible mientras el VideoWriter esta activo.
    if grabando:
        rec_x = ancho - int(280 * escala)
        rec_y = int(145 * escala)
        caja = imagen.copy()
        cv2.rectangle(caja, (rec_x, rec_y - int(34 * escala)),
                      (ancho - int(18 * escala), rec_y + int(15 * escala)), (12, 12, 28), -1)
        cv2.addWeighted(caja, 0.78, imagen, 0.22, 0, imagen)
        cv2.circle(imagen, (rec_x + int(14 * escala), rec_y - int(5 * escala)),
                   max(4, int(6 * escala)), (0, 0, 255), -1, cv2.LINE_AA)
        segundos = max(0, int(ahora - inicio_grabacion)) if inicio_grabacion is not None else 0
        horas, resto = divmod(segundos, 3600)
        minutos, segundos = divmod(resto, 60)
        texto_borde(imagen, f"GRABANDO {horas:02d}:{minutos:02d}:{segundos:02d}",
                    (rec_x + int(30 * escala), rec_y), 0.58 * escala, (0, 0, 255), 2)

    y1, y2, y3 = int(alto * 0.77), int(alto * 0.85), int(alto * 0.93)
    x = margen
    paso = int(ancho / 4.2)
    fila1 = [
        f"ACS {valor_hud('acs_v', '.3f', ' V')}",
        f"CORRIENTE {valor_hud('corriente_a', '+.2f', ' A')}",
        f"POTENCIA {valor_hud('potencia_w', '.1f', ' W')}",
        f"ESC {valor_hud('esc_us', '.0f', ' us')}",
    ]
    fila2 = [
        f"PRESION {valor_hud('presion_mbar', '.2f', ' mbar')}",
        f"TEMP {valor_hud('temperatura_c', '.2f', ' C')}",
        f"PROF {valor_hud('profundidad_m', '.2f', ' m')}",
        f"ACC X {valor_hud('ax_g')}  Y {valor_hud('ay_g')}  Z {valor_hud('az_g')} g",
    ]
    fila3 = [
        f"GYRO X {valor_hud('gx_dps', '.1f')}  Y {valor_hud('gy_dps', '.1f')}  Z {valor_hud('gz_dps', '.1f')} dps",
        f"MAG X {valor_hud('mx_ut', '.1f')}  Y {valor_hud('my_ut', '.1f')}  Z {valor_hud('mz_ut', '.1f')} uT",
    ]
    for indice, texto in enumerate(fila1):
        texto_borde(imagen, texto, (x + indice * paso, y1), 0.53 * escala, (255, 255, 255), 1)
    for indice, texto in enumerate(fila2):
        texto_borde(imagen, texto, (x + indice * paso, y2), 0.48 * escala, (220, 240, 255), 1)
    texto_borde(imagen, fila3[0], (x, y3), 0.46 * escala, (220, 240, 255), 1)
    texto_borde(imagen, fila3[1], (x + int(ancho * 0.52), y3), 0.46 * escala, (220, 240, 255), 1)
    return imagen


def dibujar_panel_grafica(imagen, rectangulo, titulo, series, ahora):
    """Dibuja series con eje X temporal sobre una ventana movil de 60 segundos."""
    x, y, ancho, alto = rectangulo
    cv2.rectangle(imagen, (x, y), (x + ancho, y + alto), (34, 40, 48), -1)
    cv2.rectangle(imagen, (x, y), (x + ancho, y + alto), (90, 105, 118), 1)
    escala = max(0.45, min(0.75, ancho / 1000.0))
    cv2.putText(imagen, titulo, (x + 12, y + int(20 * escala / 0.55)),
                cv2.FONT_HERSHEY_SIMPLEX, escala, (235, 240, 245), 1, cv2.LINE_AA)

    margen_izq = max(44, int(ancho * 0.09))
    margen_der = max(14, int(ancho * 0.025))
    margen_arriba = max(50, int(alto * 0.31))
    margen_abajo = max(26, int(alto * 0.16))
    px0, py0 = x + margen_izq, y + margen_arriba
    px1, py1 = x + ancho - margen_der, y + alto - margen_abajo
    if px1 <= px0 or py1 <= py0:
        return

    for fraccion in (0.0, 0.25, 0.5, 0.75, 1.0):
        gy = int(py1 - fraccion * (py1 - py0))
        cv2.line(imagen, (px0, gy), (px1, gy), (62, 70, 78), 1, cv2.LINE_AA)
    cv2.line(imagen, (px0, py0), (px0, py1), (150, 160, 168), 1, cv2.LINE_AA)
    cv2.line(imagen, (px0, py1), (px1, py1), (150, 160, 168), 1, cv2.LINE_AA)

    muestras = [(instante, valores) for instante, valores in historial_telemetria
                if ahora - 60.0 <= instante <= ahora]
    puntos_por_serie = []
    valores_y = []
    for etiqueta, campo, color in series:
        puntos = [(instante, valores.get(campo)) for instante, valores in muestras
                  if valores.get(campo) is not None and math.isfinite(valores[campo])]
        puntos_por_serie.append((etiqueta, color, puntos))
        valores_y.extend(valor for _, valor in puntos)

    if not valores_y:
        cv2.putText(imagen, "Esperando muestras...", (px0 + 8, (py0 + py1) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, escala * 0.8, (180, 190, 198), 1, cv2.LINE_AA)
        return

    minimo, maximo = min(valores_y), max(valores_y)
    rango = maximo - minimo
    if rango < 1e-6:
        rango = max(1.0, abs(maximo) * 0.1)
    minimo -= rango * 0.12
    maximo += rango * 0.12
    rango = maximo - minimo
    cv2.putText(imagen, f"{maximo:.2f}", (x + 3, py0 + 5),
                cv2.FONT_HERSHEY_SIMPLEX, escala * 0.62, (190, 200, 207), 1, cv2.LINE_AA)
    cv2.putText(imagen, f"{minimo:.2f}", (x + 3, py1),
                cv2.FONT_HERSHEY_SIMPLEX, escala * 0.62, (190, 200, 207), 1, cv2.LINE_AA)

    inicio = ahora - 60.0
    for etiqueta, color, puntos in puntos_por_serie:
        pixeles = []
        for instante, valor in puntos:
            px = int(px0 + max(0.0, min(1.0, (instante - inicio) / 60.0)) * (px1 - px0))
            py = int(py1 - (valor - minimo) / rango * (py1 - py0))
            pixeles.append((px, py))
        if len(pixeles) > 1:
            cv2.polylines(imagen, [np.asarray(pixeles, dtype=np.int32)], False,
                          color, max(1, int(2 * escala)), cv2.LINE_AA)
        elif pixeles:
            cv2.circle(imagen, pixeles[0], max(2, int(3 * escala)), color, -1, cv2.LINE_AA)

    cv2.putText(imagen, "-60 s", (px0, y + alto - 5), cv2.FONT_HERSHEY_SIMPLEX,
                escala * 0.62, (190, 200, 207), 1, cv2.LINE_AA)
    cv2.putText(imagen, "-30 s", ((px0 + px1) // 2 - 20, y + alto - 5),
                cv2.FONT_HERSHEY_SIMPLEX, escala * 0.62, (190, 200, 207), 1, cv2.LINE_AA)
    cv2.putText(imagen, "ahora", (px1 - 42, y + alto - 5), cv2.FONT_HERSHEY_SIMPLEX,
                escala * 0.62, (190, 200, 207), 1, cv2.LINE_AA)

    leyenda_x = px0 + 8
    for etiqueta, color, puntos in puntos_por_serie:
        if not puntos:
            continue
        cv2.line(imagen, (leyenda_x, y + int(40 * escala / 0.55)),
                 (leyenda_x + 16, y + int(40 * escala / 0.55)), color, 2, cv2.LINE_AA)
        cv2.putText(imagen, etiqueta, (leyenda_x + 21, y + int(44 * escala / 0.55)),
                    cv2.FONT_HERSHEY_SIMPLEX, escala * 0.58, color, 1, cv2.LINE_AA)
        leyenda_x += max(100, int(90 * escala / 0.55))


def dibujar_graficas(ahora):
    """Vista de tendencias en vivo; el CSV conserva todas las columnas de telemetria."""
    imagen = np.zeros((ALTO_PANTALLA, ANCHO_PANTALLA, 3), dtype=np.uint8)
    imagen[:] = (20, 25, 30)
    escala = max(0.7, min(1.2, ANCHO_PANTALLA / 1600.0))
    cv2.putText(imagen, "TELEMETRIA EN TIEMPO REAL", (int(28 * escala), int(42 * escala)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9 * escala, (0, 220, 255), 2, cv2.LINE_AA)
    estado = "OPERANDO" if operando else "LISTO"
    if logs_activos and respaldo_log_activo:
        registro = "CSV + RESPALDO ACTIVOS"
    elif logs_activos:
        registro = "CSV ACTIVO, SIN RESPALDO"
    else:
        registro = "CSV NO DISPONIBLE"
    estado_valvula = "ON" if valvula_activa else "OFF"
    cv2.putText(imagen, f"MODO: {estado}   |   VALVULA: {estado_valvula}   |   {registro}   |   VENTANA: 60 s",
                (int(28 * escala), int(76 * escala)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55 * escala, (200, 220, 225), 1, cv2.LINE_AA)

    margen = max(12, int(18 * escala))
    y_inicio = int(95 * escala)
    separacion_x = margen
    separacion_y = margen
    ancho_panel = (ANCHO_PANTALLA - 3 * margen) // 2
    paneles = [
        ("Bateria (V)", [("V bateria", "bateria_v", (50, 220, 255))]),
        ("Corriente del motor (A)", [("A motor", "corriente_a", (70, 220, 100))]),
        ("Potencia electrica (W)", [("W motor", "potencia_w", (255, 180, 60))]),
        ("Presion (mbar)", [("Presion", "presion_mbar", (220, 100, 220))]),
        ("Profundidad (m)", [("Profundidad", "profundidad_m", (80, 190, 255))]),
        ("Temperatura (C)", [("Temperatura", "temperatura_c", (120, 220, 255))]),
        ("Orientacion (grados)", [
            ("Roll", "roll_deg", (80, 210, 255)),
            ("Pitch", "pitch_deg", (100, 240, 120)),
        ]),
        ("Aceleracion (g)", [
            ("X", "ax_g", (80, 210, 255)),
            ("Y", "ay_g", (100, 240, 120)),
            ("Z", "az_g", (255, 190, 80)),
        ]),
    ]
    alto_panel = max(1, (ALTO_PANTALLA - y_inicio - 5 * margen) // 4)
    for indice, (titulo, series) in enumerate(paneles):
        fila, columna = divmod(indice, 2)
        x = margen + columna * (ancho_panel + separacion_x)
        y = y_inicio + fila * (alto_panel + separacion_y)
        dibujar_panel_grafica(imagen, (x, y, ancho_panel, alto_panel), titulo, series, ahora)

    pie = "G: volver al HUD   |   R / boton 10: grabar video   |   CSV se guarda automaticamente   |   Q: salir"
    cv2.putText(imagen, pie, (margen, ALTO_PANTALLA - max(8, int(10 * escala))),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48 * escala, (180, 195, 205), 1, cv2.LINE_AA)
    return imagen


def conmutar_grabacion():
    """Activa/desactiva la grabacion del HUD con la tecla R."""
    global grabando, video_writer, inicio_grabacion
    if cv2 is None:
        return
    if not grabando:
        os.makedirs(CARPETA_VIDEOS, exist_ok=True)
        ruta = os.path.join(CARPETA_VIDEOS, time.strftime("video_%Y%m%d_%H%M%S.mp4"))
        codec = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(ruta, codec, 30, (ANCHO_PANTALLA, ALTO_PANTALLA))
        grabando = video_writer.isOpened()
        if not grabando:
            video_writer.release()
            video_writer = None
            inicio_grabacion = None
            print("No se pudo iniciar la grabacion del HUD.")
        else:
            inicio_grabacion = time.monotonic()
            print(f"Grabacion iniciada: {os.path.abspath(ruta)}")
    else:
        grabando = False
        inicio_grabacion = None
        video_writer.release()
        video_writer = None
        print("Grabacion del HUD finalizada.")


pygame.init()
pygame.joystick.init()
pygame.display.set_mode((1, 1), pygame.HIDDEN)

tiene_mando = pygame.joystick.get_count() > 0
if tiene_mando:
    joystick = pygame.joystick.Joystick(0)
    print(f"Control conectado: {joystick.get_name()}")
else:
    print("No se detecto un mando; conectare el ESP32 y dejare el vehiculo en stop.")

try:
    puerto = serial.Serial(PUERTO_COM, BAUDIOS, timeout=0.1, write_timeout=1)
except serial.SerialException as error:
    print(f"No fue posible abrir {PUERTO_COM}: {error}")
    print("Cierre el Monitor Serial de Arduino y confirme el puerto COM.")
    pygame.quit()
    sys.exit(1)


def enviar(comando):
    puerto.write((comando + "\n").encode("ascii"))
    puerto.flush()


def esperar_esp32_listo():
    """Espera una respuesta reconocible del sketch, incluso en versiones anteriores."""
    limite = time.monotonic() + ESPERA_ESP32_S
    siguiente_sondeo = 0.0
    print(f"Conectando con ESP32 en {PUERTO_COM}...")

    while time.monotonic() < limite:
        ahora = time.monotonic()
        if ahora >= siguiente_sondeo:
            enviar("ayuda")
            siguiente_sondeo = ahora + 1.0

        if puerto.in_waiting:
            respuesta = puerto.readline().decode("ascii", errors="ignore").strip()
            if respuesta:
                actualizar_telemetria(respuesta)
                print(f"ESP32: {respuesta}")
                if respuesta == "CONTROL_LISTO" or respuesta.startswith("Comandos:"):
                    return True
        else:
            time.sleep(0.01)

    return False


try:
    if not esperar_esp32_listo():
        print("El ESP32 no respondio. Revise que tenga cargado el sketch actualizado.")
        try:
            enviar("stop")
        except (serial.SerialException, OSError):
            pass
        puerto.close()
        pygame.quit()
        sys.exit(1)
except (serial.SerialException, OSError) as error:
    print(f"Fallo la comunicacion serial: {error}")
    puerto.close()
    pygame.quit()
    sys.exit(1)

enviar("stop")
if not tiene_mando:
    puerto.close()
    pygame.quit()
    sys.exit(1)

preparar_interfaz()


def leer_eje(indice):
    if indice is None or indice < 0 or indice >= joystick.get_numaxes():
        return 0.0
    return joystick.get_axis(indice)


ultimo_comando = {"esc": None, "agua": None}
ultimo_envio = {"esc": 0.0, "agua": 0.0}


def enviar_movimiento(canal, comando):
    ahora = time.monotonic()
    es_reposo = comando in ("esc_off", "agua_off")
    if (ultimo_comando[canal] != comando or
            (not es_reposo and ahora - ultimo_envio[canal] >= REENVIO_MOVIMIENTO_S)):
        enviar(comando)
        ultimo_comando[canal] = comando
        ultimo_envio[canal] = ahora


def controlar_esc():
    """Palanca izquierda vertical: arriba avanza y abajo retrocede."""
    y = leer_eje(EJE_IZQUIERDO_Y)
    if abs(y) <= DEADZONE:
        enviar_movimiento("esc", "esc_off")
        return

    intensidad = (abs(y) - DEADZONE) / (1.0 - DEADZONE)
    if ESC_REVERSIBLE:
        # En Pygame, la palanca hacia arriba normalmente entrega un valor negativo.
        pulso = round(PULSO_ESC_NEUTRO - y * 500 * intensidad)
        enviar_movimiento("esc", f"esc {limitar(pulso, 1000, 2000)}")
    elif y < 0:
        pulso = round(1000 + 1000 * intensidad)
        enviar_movimiento("esc", f"esc {limitar(pulso, 1000, 2000)}")
    else:
        enviar_movimiento("esc", "esc_off")


def controlar_bombas():
    """Palanca derecha: selecciona una maniobra de agua a la vez."""
    x = leer_eje(EJE_DERECHO_X)
    y = leer_eje(EJE_DERECHO_Y)
    if max(abs(x), abs(y)) <= DEADZONE:
        enviar_movimiento("agua", "agua_off")
    elif abs(x) >= abs(y):
        enviar_movimiento("agua", "derecha" if x > 0 else "izquierda")
    else:
        enviar_movimiento("agua", "roll_derecha" if y < 0 else "roll_izquierda")


aire_1_activo = False
aire_2_activo = False
valvula_activa = False
pinza_abierta = False


def reiniciar_esp32():
    """Pide un reinicio seguro al ESP32 y vuelve a enlazar el puerto serie."""
    global puerto, operando, bloqueo_hasta_reposo
    global aire_1_activo, aire_2_activo, valvula_activa, pinza_abierta

    try:
        enviar("reiniciar")
    except (serial.SerialException, OSError) as error:
        print(f"No se pudo enviar el reinicio al ESP32: {error}")
        return False

    operando = False
    bloqueo_hasta_reposo = False
    aire_1_activo = aire_2_activo = False
    valvula_activa = False
    pinza_abierta = False
    ultimo_comando["esc"] = ultimo_comando["agua"] = None
    print("Reinicio solicitado; el sketch apagara actuadores y pondra el ESC en neutro.")

    # Deja que el sketch vacie el mensaje serie y reinicie antes de reabrir COM.
    time.sleep(0.7)
    try:
        if puerto.is_open:
            puerto.close()
    except (serial.SerialException, OSError):
        pass

    limite = time.monotonic() + 20.0
    while time.monotonic() < limite:
        try:
            puerto = serial.Serial(PUERTO_COM, BAUDIOS, timeout=0.1, write_timeout=1)
            break
        except (serial.SerialException, OSError):
            time.sleep(0.5)
    else:
        print(f"No se pudo reconectar con el ESP32 en {PUERTO_COM}; revisa el puerto USB.")
        return False

    if not esperar_esp32_listo():
        print("El ESP32 reinicio, pero no respondio por el puerto serie.")
        if puerto.is_open:
            puerto.close()
        return False

    enviar("stop")
    print("ESP32 reiniciado y conectado; el sistema queda en LISTO y el ESC en neutro.")
    return True


def alternar_valvula():
    """Control independiente de la electroválvula con el botón 7."""
    global valvula_activa
    valvula_activa = not valvula_activa
    enviar("valvula_on" if valvula_activa else "valvula_off")
    print(f"Electrovalvula {'ACTIVADA' if valvula_activa else 'DESACTIVADA'}.")


def alternar_aire(numero):
    global aire_1_activo, aire_2_activo
    if numero == 1:
        aire_1_activo = not aire_1_activo
        enviar("aire1_on" if aire_1_activo else "aire1_off")
    else:
        aire_2_activo = not aire_2_activo
        enviar("aire2_on" if aire_2_activo else "aire2_off")


print("Conectado. Start alterna entre LISTO y OPERANDO.")
print("En OPERANDO: palancas ESC/agua; X/Y bombas de aire; B alterna abrir/cerrar pinza; boton 7 electrovalvula.")
print("A reinicia el ESP32 de forma segura; tambien funciona desde OPERANDO.")

reloj = pygame.time.Clock()
ejecutando = True
operando = False
bloqueo_hasta_reposo = False
iniciar_logs()

try:
    while ejecutando:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                ejecutando = False
            elif evento.type == pygame.JOYDEVICEREMOVED:
                print("Se desconecto el mando; enviando stop.")
                ejecutando = False
            elif evento.type == pygame.JOYBUTTONDOWN:
                if evento.button == BOTON_GRABAR:
                    conmutar_grabacion()
                elif evento.button == BOTON_START:
                    aire_1_activo = aire_2_activo = False
                    valvula_activa = False
                    ultimo_comando["esc"] = ultimo_comando["agua"] = None
                    if operando:
                        operando = False
                        bloqueo_hasta_reposo = False
                        enviar("stop")
                        print("Modo LISTO; actuadores detenidos.")
                    else:
                        enviar("iniciar")
                        operando = True
                        bloqueo_hasta_reposo = True
                        print("Modo OPERANDO; centre las palancas para habilitar movimiento.")
                elif evento.button == BOTON_REINICIAR_ESP32:
                    reiniciar_esp32()
                elif operando and evento.button == BOTON_X:
                    alternar_aire(1)
                elif operando and evento.button == BOTON_Y:
                    alternar_aire(2)
                elif operando and evento.button == BOTON_VALVULA:
                    alternar_valvula()
                elif operando and evento.button == BOTON_B:
                    pinza_abierta = not pinza_abierta
                    enviar("abrir" if pinza_abierta else "cerrar")
                    print(f"Pinza {'ABIERTA' if pinza_abierta else 'CERRADA'}.")

        if ejecutando and operando:
            if bloqueo_hasta_reposo:
                ejes = (leer_eje(EJE_IZQUIERDO_Y), leer_eje(EJE_DERECHO_X), leer_eje(EJE_DERECHO_Y))
                if max(abs(eje) for eje in ejes) <= DEADZONE:
                    bloqueo_hasta_reposo = False
            else:
                controlar_esc()
                controlar_bombas()

        while puerto.in_waiting:
            respuesta = puerto.readline().decode("ascii", errors="ignore").strip()
            if respuesta:
                actualizar_telemetria(respuesta)
                print(f"ESP32: {respuesta}")

        if INTERFAZ_ACTIVA:
            ahora = time.monotonic()
            imagen_hud = dibujar_graficas(ahora) if vista_graficas else dibujar_hud(operando, ahora)
            if grabando and video_writer is not None:
                video_writer.write(imagen_hud)
            cv2.imshow(NOMBRE_VENTANA, imagen_hud)
            tecla = cv2.waitKey(1) & 0xFF
            if tecla in (ord("q"), ord("Q")):
                ejecutando = False
            elif tecla in (ord("r"), ord("R")):
                conmutar_grabacion()
            elif tecla in (ord("g"), ord("G")):
                vista_graficas = not vista_graficas
                print("Vista de graficas en vivo." if vista_graficas else "Vista HUD de camara.")

        reloj.tick(60)
except KeyboardInterrupt:
    pass
except (serial.SerialException, OSError) as error:
    print(f"Se perdio la comunicacion serial: {error}")
finally:
    try:
        enviar("stop")
    except (serial.SerialException, OSError):
        pass
    if puerto.is_open:
        puerto.close()
    if video_writer is not None:
        video_writer.release()
    cerrar_logs()
    if captura is not None:
        captura.release()
    if cv2 is not None:
        cv2.destroyAllWindows()
    pygame.quit()
