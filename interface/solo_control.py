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
# Valores provisionales para probar la secuencia de ascenso; ajustar tras medir el vehiculo.
TIEMPO_PRECARGA_EMERGENCIA_S = 4.0
TIEMPO_PURGA_EMERGENCIA_S = 10.0

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
ultimo_muestreo_esp32_ms = None
historial_telemetria = deque(maxlen=600)  # Ultimos 60 s a 10 Hz.
# Escalas fijas para que las variaciones pequeñas no ocupen toda la grafica.
# La potencia se limita a 400 W, el maximo teorico para 20 V y 20 A.
ESCALAS_GRAFICAS = {
    "bateria_v": (0.0, 20.0, 5.0),
    "corriente_a": (0.0, 20.0, 5.0),
    "potencia_w": (0.0, 400.0, 100.0),
    "presion_mbar": (0.0, 1500.0, 300.0),
    "profundidad_m": (0.0, 1.5, 0.5),
    "temperatura_c": (0.0, 40.0, 10.0),
}
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
estado_emergencia = "INACTIVA"
emergencia_activa = False
inicio_emergencia_monotonic = None
inicio_purga_emergencia_monotonic = None
BOTONES_HMI = {}
CLICK_HMI_PENDIENTE = None
POSICION_MOUSE_HMI = (-1, -1)

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
    """Acepta filas con tiempo ESP32 y formatos anteriores sin esa marca de tiempo."""
    global ultima_telemetria, fallo_reportado_esp32, ultimo_muestreo_esp32_ms
    texto = linea.strip()
    if "ERROR:" in texto.upper():
        fallo_reportado_esp32 = True
    if texto.startswith("ESP32:"):
        texto = texto[len("ESP32:"):].strip()
    columnas = texto.split()
    tiempo_muestreo_esp32_ms = None
    if len(columnas) in (21, len(CAMPOS_TELEMETRIA) + 1):
        try:
            tiempo_muestreo_esp32_ms = int(columnas.pop(0))
        except ValueError:
            return False
    # El firmware previo enviaba 20 columnas; el que integra BNO055 agrega
    # cuatro estados de calibracion. El firmware actual antepone el tiempo ESP32.
    # Rechaza encabezados y mensajes parciales.
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
    intervalo_muestreo_esp32_ms = None
    if tiempo_muestreo_esp32_ms is not None:
        if (ultimo_muestreo_esp32_ms is not None and
                tiempo_muestreo_esp32_ms >= ultimo_muestreo_esp32_ms):
            intervalo_muestreo_esp32_ms = tiempo_muestreo_esp32_ms - ultimo_muestreo_esp32_ms
        ultimo_muestreo_esp32_ms = tiempo_muestreo_esp32_ms
    else:
        ultimo_muestreo_esp32_ms = None
    registrar_fila_log(nuevos, ahora, tiempo_muestreo_esp32_ms, intervalo_muestreo_esp32_ms)
    return True


def iniciar_logs():
    """Abre CSV de sesion y una copia espejo en la carpeta de respaldo."""
    global inicio_logs_monotonic, logs_activos, respaldo_log_activo, ruta_log_respaldo
    global ultimo_muestreo_esp32_ms
    carpeta_respaldo = os.path.join(CARPETA_LOGS, "Respaldo")
    try:
        os.makedirs(CARPETA_LOGS, exist_ok=True)
        os.makedirs(carpeta_respaldo, exist_ok=True)
    except OSError as error:
        print(f"No se pudo crear la carpeta de logs: {error}")
        return False
    marca = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    nombre = f"telemetria_{marca}.csv"
    encabezado = ["fecha_hora_local", "tiempo_s"] + list(COLUMNAS_LOG)
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
    ultimo_muestreo_esp32_ms = None
    logs_activos = bool(escritores_log)
    respaldo_log_activo = any(etiqueta == "respaldo" for _, _, _, etiqueta in escritores_log)
    if logs_activos:
        print("Captura CSV activa." if not respaldo_log_activo else
              "Captura CSV activa con copia espejo de respaldo.")
    return logs_activos


def registrar_fila_log(valores, ahora, tiempo_muestreo_esp32_ms=None, intervalo_muestreo_esp32_ms=None):
    """Escribe y vacia cada muestra para conservar los datos ante un cierre abrupto."""
    global logs_activos, respaldo_log_activo
    if not logs_activos or inicio_logs_monotonic is None:
        return
    marca_hora = datetime.now().astimezone().isoformat(timespec="milliseconds")
    tiempo_s = f"{ahora - inicio_logs_monotonic:.3f}"
    fila = [marca_hora, tiempo_s]
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
    global logs_activos, respaldo_log_activo, inicio_logs_monotonic, ultimo_muestreo_esp32_ms
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
    inicio_logs_monotonic = None
    ultimo_muestreo_esp32_ms = None


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



def evento_mouse_hmi(evento, x, y, flags, parametro):
    """Registra clics sobre los botones dibujados en la ventana OpenCV."""
    global CLICK_HMI_PENDIENTE, POSICION_MOUSE_HMI
    POSICION_MOUSE_HMI = (x, y)
    if evento == cv2.EVENT_LBUTTONDOWN:
        for nombre, (bx, by, bw, bh) in tuple(BOTONES_HMI.items()):
            if bx <= x <= bx + bw and by <= y <= by + bh:
                CLICK_HMI_PENDIENTE = nombre
                break


def dibujar_boton_hmi(imagen, nombre, etiqueta, rectangulo, habilitado, color):
    x, y, ancho, alto = rectangulo
    if habilitado:
        BOTONES_HMI[nombre] = rectangulo
    mouse_x, mouse_y = POSICION_MOUSE_HMI
    sobre = habilitado and x <= mouse_x <= x + ancho and y <= mouse_y <= y + alto
    if not habilitado:
        relleno = (62, 66, 70)
        borde = (105, 110, 115)
        color_texto = (150, 155, 160)
    elif sobre:
        relleno = tuple(min(255, int(componente * 1.16 + 12)) for componente in color)
        borde = (245, 245, 245)
        color_texto = (255, 255, 255)
    else:
        relleno = color
        borde = tuple(min(255, componente + 35) for componente in color)
        color_texto = (255, 255, 255)
    cv2.rectangle(imagen, (x, y), (x + ancho, y + alto), relleno, -1, cv2.LINE_AA)
    cv2.rectangle(imagen, (x, y), (x + ancho, y + alto), borde, max(1, int(alto * 0.035)), cv2.LINE_AA)
    fuente = cv2.FONT_HERSHEY_SIMPLEX
    escala_texto = min(0.62, alto / 66.0)
    (ancho_texto, alto_texto), _ = cv2.getTextSize(etiqueta, fuente, escala_texto, 2)
    if ancho_texto > ancho - 10:
        escala_texto *= max(0.35, (ancho - 10) / max(1, ancho_texto))
        (ancho_texto, alto_texto), _ = cv2.getTextSize(etiqueta, fuente, escala_texto, 2)
    origen = (x + (ancho - ancho_texto) // 2, y + (alto + alto_texto) // 2)
    cv2.putText(imagen, etiqueta, origen, fuente, escala_texto, color_texto, 2, cv2.LINE_AA)


def dibujar_controles_hmi(imagen, graficas=False):
    """Agrega controles clicables en los costados, sin cubrir el centro de la camara."""
    BOTONES_HMI.clear()
    ancho_imagen, alto_imagen = imagen.shape[1], imagen.shape[0]
    if graficas:
        escala = max(0.65, min(1.0, ancho_imagen / 1600.0))
        margen = max(8, int(10 * escala))
        separacion = max(4, int(7 * escala))
        alto_boton = max(38, int(43 * escala))
        x_inicio = int(ancho_imagen * 0.55)
        ancho_total = ancho_imagen - x_inicio - margen
        ancho_boton = (ancho_total - 3 * separacion) // 4
        y_boton = max(6, int(8 * escala))
        rectangulos = [
            (x_inicio + indice * (ancho_boton + separacion), y_boton, ancho_boton, alto_boton)
            for indice in range(4)
        ]
    else:
        escala = max(0.8, min(1.25, alto_imagen / 800.0))
        ancho_lateral = int(ancho_imagen * 0.24)
        margen = max(8, int(14 * escala))
        separacion = max(6, int(9 * escala))
        alto_boton = max(42, int(44 * escala))
        y_boton = alto_imagen - alto_boton - max(8, int(10 * escala))
        ancho_util = ancho_lateral - 2 * margen - separacion
        ancho_izquierdo = ancho_util // 2
        rectangulos = [
            (margen, y_boton, ancho_izquierdo, alto_boton),
            (margen + ancho_izquierdo + separacion, y_boton, ancho_util - ancho_izquierdo, alto_boton),
        ]
        x_derecha = ancho_imagen - ancho_lateral + margen
        ancho_emergencia = int(ancho_util * 0.68)
        rectangulos.extend([
            (x_derecha, y_boton, ancho_emergencia, alto_boton),
            (x_derecha + ancho_emergencia + separacion, y_boton, ancho_util - ancho_emergencia, alto_boton),
        ])

    if graficas:
        nombres = ("start", "stop", "emergencia", "rearmar")
    else:
        nombres = ("start", "stop", "emergencia", "rearmar")
    emergencia_etiqueta = "EMERGENCIA"
    if emergencia_activa and estado_emergencia in ("PRECARGA", "PURGA"):
        emergencia_etiqueta = "ASCENSO ACTIVO"
    elif emergencia_activa and estado_emergencia == "FINALIZADA":
        emergencia_etiqueta = "ASCENSO LISTO"

    configuracion = (
        ("start", "START", not operando and not emergencia_activa, (34, 135, 40)),
        ("stop", "STOP", operando and not emergencia_activa, (0, 130, 220)),
        ("emergencia", emergencia_etiqueta, not emergencia_activa, (20, 20, 210)),
        ("rearmar", "REARMAR", emergencia_activa and estado_emergencia == "FINALIZADA", (145, 105, 20)),
    )
    for (nombre, etiqueta, habilitado, color), rectangulo in zip(configuracion, rectangulos):
        dibujar_boton_hmi(imagen, nombre, etiqueta, rectangulo, habilitado, color)


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
        cv2.setMouseCallback(NOMBRE_VENTANA, evento_mouse_hmi)
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
    alto_datos_laterales = int(alto * 0.70)
    ancho_panel_lateral = int(ancho * 0.24)
    capa = imagen.copy()
    cv2.rectangle(capa, (0, 0), (ancho, int(90 * escala)), (8, 16, 22), -1)
    cv2.rectangle(capa, (0, alto_datos_laterales), (ancho_panel_lateral, alto), (8, 16, 22), -1)
    cv2.rectangle(capa, (ancho - ancho_panel_lateral, alto_datos_laterales),
                  (ancho, alto), (8, 16, 22), -1)
    cv2.addWeighted(capa, 0.68, imagen, 0.32, 0, imagen)

    margen = int(28 * escala)
    vivo = ultima_telemetria > 0 and ahora - ultima_telemetria <= TIEMPO_SIN_TELEMETRIA_S
    fallas_sensores = sensores_fallando() if vivo else []
    error_sensores = bool(fallas_sensores)
    bateria_baja = telemetria.get("bateria_v") is not None and telemetria["bateria_v"] < 10.5
    estado = "EMERGENCIA" if emergencia_activa else ("OPERANDO" if operando else "LISTO")
    color_modo = (0, 0, 255) if emergencia_activa else ((0, 210, 255) if operando else (255, 220, 0))
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
        diferencia_pitch = (pitch_objetivo - pitch_hud_filtrado + 180.0) % 360.0 - 180.0
        pitch_hud_filtrado += alpha_actitud * diferencia_pitch
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

    datos_izquierda = [
        f"ACS {valor_hud('acs_v', '.3f', ' V')}",
        f"CORRIENTE {valor_hud('corriente_a', '+.2f', ' A')}",
        f"POTENCIA {valor_hud('potencia_w', '.1f', ' W')}",
        f"ESC {valor_hud('esc_us', '.0f', ' us')}",
        f"GYRO X {valor_hud('gx_dps', '.1f')}  Y {valor_hud('gy_dps', '.1f')}  Z {valor_hud('gz_dps', '.1f')} dps",
    ]
    datos_derecha = [
        f"PRESION {valor_hud('presion_mbar', '.2f', ' mbar')}",
        f"TEMP {valor_hud('temperatura_c', '.2f', ' C')}",
        f"PROF {valor_hud('profundidad_m', '.2f', ' m')}",
        f"ACC X {valor_hud('ax_g')}  Y {valor_hud('ay_g')}  Z {valor_hud('az_g')} g",
        f"MAG X {valor_hud('mx_ut', '.1f')}  Y {valor_hud('my_ut', '.1f')}  Z {valor_hud('mz_ut', '.1f')} uT",
    ]
    inicio_datos_y = alto_datos_laterales + int(12 * escala)
    separacion_datos_y = int(36 * escala)
    x_datos_derecha = ancho - ancho_panel_lateral + margen
    for indice, texto in enumerate(datos_izquierda):
        texto_borde(imagen, texto, (margen, inicio_datos_y + indice * separacion_datos_y),
                    0.46 * escala, (220, 240, 255), 1)
    for indice, texto in enumerate(datos_derecha):
        texto_borde(imagen, texto, (x_datos_derecha, inicio_datos_y + indice * separacion_datos_y),
                    0.46 * escala, (220, 240, 255), 1)
    return imagen


def suavizar_serie_grafica(puntos, ventana=5, factor=0.25):
    """Reduce ruido visual con mediana y promedio exponencial, sin tocar los datos guardados."""
    recientes = deque(maxlen=ventana)
    suavizados = []
    filtrado = None
    for instante, valor in puntos:
        recientes.append(valor)
        ordenados = sorted(recientes)
        mediana = ordenados[len(ordenados) // 2]
        filtrado = mediana if filtrado is None else factor * mediana + (1.0 - factor) * filtrado
        suavizados.append((instante, filtrado))
    return suavizados


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

    config_escala = ESCALAS_GRAFICAS.get(series[0][1]) if series else None
    if config_escala:
        escala_min, escala_max, paso_marca = config_escala
        marcas = [escala_min + indice * paso_marca
                  for indice in range(int(round((escala_max - escala_min) / paso_marca)) + 1)]
        fracciones = [(valor - escala_min) / (escala_max - escala_min) for valor in marcas]
    else:
        marcas = []
        fracciones = (0.0, 0.25, 0.5, 0.75, 1.0)

    for fraccion in fracciones:
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
        if config_escala and puntos:
            puntos = suavizar_serie_grafica(puntos)
        puntos_por_serie.append((etiqueta, color, puntos))
        valores_y.extend(valor for _, valor in puntos)

    if not valores_y:
        cv2.putText(imagen, "Esperando muestras...", (px0 + 8, (py0 + py1) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, escala * 0.8, (180, 190, 198), 1, cv2.LINE_AA)
        return

    if config_escala:
        minimo, maximo, _ = config_escala
    else:
        minimo, maximo = min(valores_y), max(valores_y)
        rango = maximo - minimo
        if rango < 1e-6:
            rango = max(1.0, abs(maximo) * 0.1)
        minimo -= rango * 0.12
        maximo += rango * 0.12
    rango = maximo - minimo
    if config_escala:
        for valor, fraccion in zip(marcas, fracciones):
            gy = int(py1 - fraccion * (py1 - py0))
            etiqueta_eje = f"{valor:g}"
            base_y = min(gy + 4, py1 - 2)
            cv2.putText(imagen, etiqueta_eje, (x + 3, base_y),
                        cv2.FONT_HERSHEY_SIMPLEX, escala * 0.62, (190, 200, 207), 1, cv2.LINE_AA)
    else:
        cv2.putText(imagen, f"{maximo:.2f}", (x + 3, py0 + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, escala * 0.62, (190, 200, 207), 1, cv2.LINE_AA)
        cv2.putText(imagen, f"{minimo:.2f}", (x + 3, py1),
                    cv2.FONT_HERSHEY_SIMPLEX, escala * 0.62, (190, 200, 207), 1, cv2.LINE_AA)

    inicio = ahora - 60.0
    for etiqueta, color, puntos in puntos_por_serie:
        pixeles = []
        for instante, valor in puntos:
            px = int(px0 + max(0.0, min(1.0, (instante - inicio) / 60.0)) * (px1 - px0))
            valor_grafica = max(minimo, min(maximo, valor)) if config_escala else valor
            py = int(py1 - (valor_grafica - minimo) / rango * (py1 - py0))
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
    estado = "EMERGENCIA" if emergencia_activa else ("OPERANDO" if operando else "LISTO")
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
        pulso = round(PULSO_ESC_NEUTRO + y * 500 * intensidad)
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
    cerrar_logs()
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



def iniciar_desde_hmi():
    """Inicia el mismo modo normal que Start del mando, solo desde el boton HMI."""
    global operando, bloqueo_hasta_reposo
    global aire_1_activo, aire_2_activo, valvula_activa
    if operando or emergencia_activa:
        return
    aire_1_activo = aire_2_activo = False
    valvula_activa = False
    ultimo_comando["esc"] = ultimo_comando["agua"] = None
    puerto.reset_input_buffer()
    enviar("iniciar")
    operando = True
    bloqueo_hasta_reposo = True
    iniciar_logs()
    print("Inicio desde HMI; centre las palancas para habilitar movimiento.")


def detener_desde_hmi():
    """Detiene el modo normal sin cambiar la posicion mandada a la garra."""
    global operando, bloqueo_hasta_reposo
    global aire_1_activo, aire_2_activo, valvula_activa
    if not operando or emergencia_activa:
        return
    aire_1_activo = aire_2_activo = False
    valvula_activa = False
    ultimo_comando["esc"] = ultimo_comando["agua"] = None
    operando = False
    bloqueo_hasta_reposo = False
    enviar("stop")
    cerrar_logs()
    print("STOP desde HMI; control normal detenido.")


def iniciar_emergencia_hmi():
    """Detiene la maniobra y ejecuta la secuencia provisional de ascenso."""
    global operando, bloqueo_hasta_reposo
    global aire_1_activo, aire_2_activo, valvula_activa
    global emergencia_activa, estado_emergencia
    global inicio_emergencia_monotonic, inicio_purga_emergencia_monotonic
    if emergencia_activa:
        return
    operando = False
    bloqueo_hasta_reposo = False
    aire_1_activo = True
    aire_2_activo = False
    valvula_activa = False
    ultimo_comando["esc"] = ultimo_comando["agua"] = None
    enviar("stop")
    cerrar_logs()
    enviar("aire1_on")
    emergencia_activa = True
    estado_emergencia = "PRECARGA"
    inicio_emergencia_monotonic = time.monotonic()
    inicio_purga_emergencia_monotonic = None
    print(
        "EMERGENCIA HMI: ESC y maniobras detenidos; bomba de aire activa. "
        f"Valvula en {TIEMPO_PRECARGA_EMERGENCIA_S:.0f} s; "
        f"purga provisional {TIEMPO_PURGA_EMERGENCIA_S:.0f} s."
    )


def actualizar_secuencia_emergencia():
    """Avanza la temporizacion sin bloquear la ventana ni los clics de la HMI."""
    global aire_1_activo, valvula_activa, estado_emergencia
    global inicio_purga_emergencia_monotonic
    if not emergencia_activa:
        return
    ahora = time.monotonic()
    if estado_emergencia == "PRECARGA" and inicio_emergencia_monotonic is not None:
        if ahora - inicio_emergencia_monotonic >= TIEMPO_PRECARGA_EMERGENCIA_S:
            enviar("valvula_on")
            valvula_activa = True
            estado_emergencia = "PURGA"
            inicio_purga_emergencia_monotonic = ahora
            print("EMERGENCIA HMI: electroválvula activada.")
    elif estado_emergencia == "PURGA" and inicio_purga_emergencia_monotonic is not None:
        if ahora - inicio_purga_emergencia_monotonic >= TIEMPO_PURGA_EMERGENCIA_S:
            enviar("valvula_off")
            enviar("aire1_off")
            valvula_activa = False
            aire_1_activo = False
            estado_emergencia = "FINALIZADA"
            print("EMERGENCIA HMI: bomba y electroválvula apagadas; rearme requerido.")


def rearmar_emergencia_hmi():
    """Desbloquea unicamente despues de terminar la secuencia; no inicia el control."""
    global emergencia_activa, estado_emergencia
    global inicio_emergencia_monotonic, inicio_purga_emergencia_monotonic
    if not emergencia_activa or estado_emergencia != "FINALIZADA":
        return
    enviar("stop")
    emergencia_activa = False
    estado_emergencia = "INACTIVA"
    inicio_emergencia_monotonic = None
    inicio_purga_emergencia_monotonic = None
    print("Emergencia rearmada; pulse START para operar.")


def procesar_click_hmi():
    global CLICK_HMI_PENDIENTE
    accion = CLICK_HMI_PENDIENTE
    CLICK_HMI_PENDIENTE = None
    if accion == "start":
        iniciar_desde_hmi()
    elif accion == "stop":
        detener_desde_hmi()
    elif accion == "emergencia":
        iniciar_emergencia_hmi()
    elif accion == "rearmar":
        rearmar_emergencia_hmi()


print("Conectado. Start alterna entre LISTO y OPERANDO.")
print("En OPERANDO: palancas ESC/agua; X/Y bombas de aire; B alterna abrir/cerrar pinza; boton 7 electrovalvula.")
print("A reinicia el ESP32 de forma segura; tambien funciona desde OPERANDO.")

reloj = pygame.time.Clock()
ejecutando = True
operando = False
bloqueo_hasta_reposo = False

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
                elif evento.button == BOTON_START and not emergencia_activa:
                    aire_1_activo = aire_2_activo = False
                    valvula_activa = False
                    ultimo_comando["esc"] = ultimo_comando["agua"] = None
                    if operando:
                        operando = False
                        bloqueo_hasta_reposo = False
                        enviar("stop")
                        cerrar_logs()
                        print("Modo LISTO; actuadores detenidos.")
                    else:
                        puerto.reset_input_buffer()
                        enviar("iniciar")
                        operando = True
                        bloqueo_hasta_reposo = True
                        iniciar_logs()
                        print("Modo OPERANDO; centre las palancas para habilitar movimiento.")
                elif evento.button == BOTON_REINICIAR_ESP32 and not emergencia_activa:
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

        if ejecutando:
            procesar_click_hmi()
            actualizar_secuencia_emergencia()

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
            dibujar_controles_hmi(imagen_hud, vista_graficas)
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
