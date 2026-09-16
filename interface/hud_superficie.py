import cv2
import numpy as np
import ctypes
import time
import serial
import math
import os
import pygame

# --- CONFIGURACIÓN DE DESFASES (CALIBRACIÓN) ---
OFFSET_PITCH = 3.5
OFFSET_ROLL = 2.6

# --- CONFIGURACIÓN GENERAL ---
PUERTO_COM = 'COM3' 
BAUDIOS = 115200
VAL_CONTRASTE = 1.24
VAL_BRILLO = 0
VAL_SATURACION = 1.70
camera = 0

# --- CONFIGURACIÓN DEL MANDO ---
BTN_SELECT = 6  
BTN_START = 7   

# --- CARPETA DE GRABACIÓN ---
CARPETA_VIDEOS = "Grabaciones_HUD"
if not os.path.exists(CARPETA_VIDEOS):
    os.makedirs(CARPETA_VIDEOS)
RUTA_ABSOLUTA = os.path.abspath(CARPETA_VIDEOS)

# --- AJUSTE DE ZONA MUERTA ---
DEADZONE = 0.25

DIAS = ["Lunes", "Martes", "Miercoles", "Jueves", "Viernes", "Sabado", "Domingo"]
MESES = ["Enero", "Febrero", "Marzo", "Abril", "Mayo", "Junio", "Julio", "Agosto", "Septiembre", "Octubre", "Noviembre", "Diciembre"]

# 1. Resolución del Monitor
user32 = ctypes.windll.user32
SCREEN_W, SCREEN_H = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)

# 2. Configurar Cámara y Serial
cap = cv2.VideoCapture(camera, cv2.CAP_DSHOW)
cam_w, cam_h = 800, 600
cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_h)

try:
    ser = serial.Serial(PUERTO_COM, BAUDIOS, timeout=0.001) 
    print(f"✅ Conectado exitosamente a {PUERTO_COM}")
    SIMULACION_ACTIVA = False
except:
    print("⚠️ ADVERTENCIA: No se detectó el puerto COM. Iniciando en MODO SIMULACIÓN.")
    ser = None
    SIMULACION_ACTIVA = True

# 3. Inicializar Pygame y Joystick
pygame.init()
pygame.joystick.init()
pygame.display.init()  

joystick_detectado = False
if pygame.joystick.get_count() == 0:
    print("⚠️ ADVERTENCIA: No se detectó el control.")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"🎮 Control configurado: {joystick.get_name()}")
    joystick_detectado = True

nombre_ventana = 'Sistema Vision Pro HUD'
cv2.namedWindow(nombre_ventana, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(nombre_ventana, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# Variables de estado y persistencia
prev_frame_time = 0
datos_mpu = {"AngX": 0.0, "AngY": 0.0, "RumboZ": 0.0}
capa_hud_hd = None  

ult_x_cuerpo_hd = None
ult_y_cuerpo_hd = None

voltaje_bateria = 12.0  
porcentaje_bateria = 100
corriente_mA = 0.0

temperatura_c = 0.0
presion_bar = 0.0
profundidad_m = 0.0

modo_sistema = "STBY"      
tiempo_run_total = 0.0     
ultimo_cambio_tiempo = time.time()

tiempo_ultimo_ping_enviado = 0
tiempo_ultimo_ping_recibido = time.time()  
esperando_pong = False
ping_ms = 0
estado_conexion = "LIVE"  

grabando = False
video_writer = None  

# Estados anteriores para controlar el envío único por Serial
ultimo_enviado = None

def texto_con_borde(img, texto, posicion, fuente, escala, color, grosor=1, color_borde=(0, 0, 0)):
    x, y = posicion
    cv2.putText(img, texto, (x - 1, y), fuente, escala, color_borde, grosor, cv2.LINE_AA)
    cv2.putText(img, texto, (x + 1, y), fuente, escala, color_borde, grosor, cv2.LINE_AA)
    cv2.putText(img, texto, (x, y - 1), fuente, escala, color_borde, grosor, cv2.LINE_AA)
    cv2.putText(img, texto, (x, y + 1), fuente, escala, color_borde, grosor, cv2.LINE_AA)
    cv2.putText(img, texto, (x, y), fuente, escala, color, grosor, cv2.LINE_AA)

def dibujar_linea_discontinua(img, pt1, pt2, color, grosor, largo_segmento=10, espacio=8):
    dist = math.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])
    if dist == 0: return
    dx = (pt2[0] - pt1[0]) / dist
    dy = (pt2[1] - pt1[1]) / dist
    
    curr_dist = 0
    while curr_dist < dist:
        x_ini = int(pt1[0] + dx * curr_dist)
        y_ini = int(pt1[1] + dy * curr_dist)
        curr_dist += largo_segmento
        if curr_dist > dist: curr_dist = dist
        x_fin = int(pt1[0] + dx * curr_dist)
        y_fin = int(pt1[1] + dy * curr_dist)
        cv2.line(img, (x_ini, y_ini), (x_fin, y_fin), color, grosor, cv2.LINE_AA)
        curr_dist += espacio 

def formatear_segundos(segundos):
    hrs = int(segundos // 3600)
    mins = int((segundos % 3600) // 60)
    segs = int(segundos % 60)
    return f"{hrs:02d}:{mins:02d}:{segs:02d}"

def enviar_comando_serial(id_boton):
    global ultimo_enviado
    if id_boton == ultimo_enviado:
        return
    ultimo_enviado = id_boton
    
    cadena = f"$CTR,{id_boton}"
    if not SIMULACION_ACTIVA and ser:
        try:
            ser.write(f"{cadena}\n".encode('utf-8'))
        except:
            pass
    else:
        print(f"🤖 [SERIAL] Enviado: {cadena}")

def conmutar_grabacion():
    global grabando, video_writer
    grabando = not grabando  
    if grabando:
        nombre_archivo = time.strftime("video_%Y%m%d_%H%M%S.mp4")
        ruta_completa_video = os.path.join(CARPETA_VIDEOS, nombre_archivo)
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
        video_writer = cv2.VideoWriter(ruta_completa_video, fourcc, 30, (SCREEN_W, SCREEN_H))
        print(f"\n🎬 GRABACIÓN INICIADA -> Guardando en: {os.path.abspath(ruta_completa_video)}")
    else:
        if video_writer is not None:
            video_writer.release()
            video_writer = None
        print("\n🛑 GRABACIÓN FINALIZADA Y GUARDADA EN DISCO DURO.")

# --- BUCLE PRINCIPAL ---
ejecutando = True
while ejecutando:
    ret, frame = cap.read()
    if not ret: 
        frame = np.zeros((600, 800, 3), dtype=np.uint8)

    t_actual = time.time()

    if t_actual - tiempo_ultimo_ping_recibido > 20.0:
        estado_conexion = "LOST"
    else:
        estado_conexion = "LIVE"

    if not SIMULACION_ACTIVA and ser:
        if not esperando_pong and (t_actual - tiempo_ultimo_ping_enviado > 2.0):
            try:
                ser.write(b"pong\n")
                tiempo_ultimo_ping_enviado = t_actual
                esperando_pong = True
            except:
                pass

    if not SIMULACION_ACTIVA and ser:
        if ser.in_waiting > 0:
            try:
                while ser.in_waiting > 200:
                    ser.readline() 
                linea = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if linea.startswith("$MPU"):
                    partes = linea.split(',')
                    if len(partes) >= 7:
                        datos_mpu["AngX"] = float(partes[4]) + OFFSET_ROLL
                        datos_mpu["AngY"] = float(partes[5]) + OFFSET_PITCH
                        datos_mpu["RumboZ"] = float(partes[6].replace(';', ''))
                elif "$BATERY" in linea:
                    partes = linea.split(',')
                    if len(partes) >= 2:
                        val_str = partes[1].replace(';', '').strip()
                        voltaje_bateria = float(val_str)
                        porcentaje_bateria = int(((voltaje_bateria - 9.0) / (12.0 - 9.0)) * 100)
                        porcentaje_bateria = max(0, min(100, porcentaje_bateria)) 
                elif "$MAH" in linea:
                    partes = linea.split(',')
                    if len(partes) >= 2:
                        corriente_mA = float(partes[1].replace(';', '').strip())
                elif "$TEMP" in linea:
                    partes = linea.split(',')
                    if len(partes) >= 2:
                        temperatura_c = float(partes[1].replace(';', '').strip())
                elif "$BAR" in linea:
                    partes = linea.split(',')
                    if len(partes) >= 2:
                        presion_bar = float(partes[1].replace(';', '').strip())
                elif "$PROF" in linea:
                    partes = linea.split(',')
                    if len(partes) >= 2:
                        profundidad_m = float(partes[1].replace(';', '').strip())
                elif "$MODE" in linea:
                    partes = linea.split(',')
                    if len(partes) >= 2:
                        modo_sistema = partes[1].replace(';', '').strip().upper()
                elif "$PING" in linea:
                    tiempo_ultimo_ping_recibido = t_actual  
                    if esperando_pong:
                        ping_ms = int((t_actual - tiempo_ultimo_ping_enviado) * 1000)
                        esperando_pong = False
            except:
                pass 
    else:
        datos_mpu["AngX"] = math.sin(t_actual * 0.4) * 3.0  
        datos_mpu["AngY"] = math.cos(t_actual * 0.3) * 3.0  
        datos_mpu["RumboZ"] = (t_actual * 15) % 360          
        
        voltaje_bateria = 8.8 + (math.sin(t_actual * 0.1) + 1.0) * 1.6
        porcentaje_bateria = int(((voltaje_bateria - 9.0) / (12.0 - 9.0)) * 100)
        porcentaje_bateria = max(0, min(100, porcentaje_bateria))
        corriente_mA = 450.0 + math.sin(t_actual * 0.5) * 120.0
        temperatura_c = 18.5 + math.sin(t_actual * 0.05) * 2.0
        profundidad_m = 12.4 + math.sin(t_actual * 0.1) * 5.0
        presion_bar = 1.013 + (profundidad_m * 0.1)
        
        ciclo_modo = int(t_actual // 15) % 4
        modos_sim = ["STBY", "RUN", "CALIBRATE", "HOME"]
        modo_sistema = modos_sim[ciclo_modo]
        
        if t_actual - tiempo_ultimo_ping_enviado > 2.0:
            ping_ms = int(22 + math.sin(t_actual) * 6)
            tiempo_ultimo_ping_enviado = t_actual
            tiempo_ultimo_ping_recibido = t_actual

    # ==========================================================
    # LÓGICA DE DETECCIÓN EXACTA DEL MANDO
    # ==========================================================
    hubo_actividad = False
    boton_actual_texto = ""

    if joystick_detectado:
        pygame.event.pump()

        # 1. Botones principales digitales (0-11)
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                boton_actual_texto = f"BOTON {i}"
                hubo_actividad = True
                enviar_comando_serial(i)

        # 2. Control de la Cruceta (D-Pad) -> Arriba(12), Abajo(13), Izquierda(14), Derecha(15)
        if joystick.get_numhats() > 0:
            hx, hy = joystick.get_hat(0)
            if hx != 0 or hy != 0:
                hubo_actividad = True
                if hy == 1:
                    boton_actual_texto = "BOTON 12"
                    enviar_comando_serial(12)
                elif hy == -1:
                    boton_actual_texto = "BOTON 13"
                    enviar_comando_serial(13)
                elif hx == -1:
                    boton_actual_texto = "BOTON 14"
                    enviar_comando_serial(14)
                elif hx == 1:
                    boton_actual_texto = "BOTON 15"
                    enviar_comando_serial(15)

        # 3. Joystick Izquierdo (Ejes por defecto 0 y 1)
        num_ejes = joystick.get_numaxes()
        if num_ejes >= 2:
            x_izq = joystick.get_axis(0)
            y_izq = joystick.get_axis(1)

            if abs(x_izq) > DEADZONE or abs(y_izq) > DEADZONE:
                hubo_actividad = True
                if abs(x_izq) > abs(y_izq):
                    if x_izq > 0:
                        boton_actual_texto = "BOTON 19" 
                        enviar_comando_serial(19)
                    else:
                        boton_actual_texto = "BOTON 18" 
                        enviar_comando_serial(18)
                else:
                    if y_izq > 0:
                        boton_actual_texto = "BOTON 17" 
                        enviar_comando_serial(17)
                    else:
                        boton_actual_texto = "BOTON 16" 
                        enviar_comando_serial(16)

        # 4. Joystick Derecho Universal (Escaneo adaptativo multitarget) - CORREGIDO
        if num_ejes >= 4:
            ejes_derechos = []
            for axis_idx in range(2, num_ejes):
                val = joystick.get_axis(axis_idx)
                # Filtro estricto: omitir ejes flotantes en reposo que se clavan en -1.0 o 1.0 (Gatillos)
                if abs(val) > DEADZONE and abs(val - 1.0) > 0.08 and abs(val + 1.0) > 0.08:
                    ejes_derechos.append((axis_idx, val))
            
            if len(ejes_derechos) > 0:
                # Si se detectan múltiples movimientos, priorizar el de mayor magnitud física
                ejes_derechos.sort(key=lambda x: abs(x[1]), reverse=True)
                eje_activo, valor_activo = ejes_derechos[0]
                hubo_actividad = True
                
                # Identificación explícita según estándares de hardware comunes
                es_horizontal = eje_activo in [2, 3]
                es_vertical = eje_activo in [4, 5]
                
                # Respaldo matemático si el mando es muy particular
                if not es_horizontal and not es_vertical:
                    if eje_activo % 2 == 0:
                        es_horizontal = True
                    else:
                        es_vertical = True

                if es_horizontal:  # Movimiento Horizontal (X)
                    if valor_activo > 0:
                        boton_actual_texto = "BOTON 23"  # Derecha
                        enviar_comando_serial(23)
                    else:
                        boton_actual_texto = "BOTON 22"  # Izquierda
                        enviar_comando_serial(22)
                elif es_vertical:  # Movimiento Vertical (Y)
                    if valor_activo > 0:
                        boton_actual_texto = "BOTON 21"  # Abajo
                        enviar_comando_serial(21)
                    else:
                        boton_actual_texto = "BOTON 20"  # Arriba
                        enviar_comando_serial(20)

        # 5. Gatillos Analógicos (Mapeados como botones complementarios de acción 10 y 11 si pasan del umbral)
        if num_ejes > 4:
            if joystick.get_axis(2) > 0.6:
                boton_actual_texto = "BOTON 10"
                hubo_actividad = True
                enviar_comando_serial(10)
            if num_ejes > 5 and joystick.get_axis(5) > 0.6:
                boton_actual_texto = "BOTON 11"
                hubo_actividad = True
                enviar_comando_serial(11)

        if not hubo_actividad:
            ultimo_enviado = None

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == BTN_START:
                    conmutar_grabacion()
                elif event.button == BTN_SELECT:
                    ejecutando = False

    dt = t_actual - ultimo_cambio_tiempo
    ultimo_cambio_tiempo = t_actual
    if modo_sistema == "RUN":
        tiempo_run_total += dt

    # --- FILTRADO DE IMAGEN ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype("float32")
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * VAL_SATURACION, 0, 255)
    img = cv2.cvtColor(hsv.astype("uint8"), cv2.COLOR_HSV2BGR)
    img = cv2.convertScaleAbs(img, alpha=VAL_CONTRASTE, beta=VAL_BRILLO)
    
    suave = cv2.GaussianBlur(img, (3, 3), 0)
    img = cv2.addWeighted(img, 1.5, suave, -0.5, 0)

    alto, ancho, _ = img.shape
    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time) if (new_frame_time - prev_frame_time) > 0 else 0
    prev_frame_time = new_frame_time

    # --- ESCALADO A PANTALLA COMPLETA ---
    pantalla_negra = np.zeros((SCREEN_H, SCREEN_W, 3), dtype=np.uint8)
    factor = min(SCREEN_W / ancho, SCREEN_H / alto)
    nw, nh = int(ancho * factor), int(alto * factor)
    img_reescalada = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    x_off, y_off = (SCREEN_W - nw) // 2, (SCREEN_H - nh) // 2
    pantalla_negra[y_off:y_off+nh, x_off:x_off+nw] = img_reescalada

    if capa_hud_hd is None:
        capa_hud_hd = np.zeros_like(pantalla_negra)

    color_aa = cv2.LINE_AA
    grosor_vector = max(1, int(1.2 * factor)) 

    # --- RADAR CON RASTRO ---
    capa_hud_hd = cv2.convertScaleAbs(capa_hud_hd, alpha=0.92, beta=0)
    centro_x_hd = int(90 * factor) + x_off
    centro_y_hd = int(110 * factor) + y_off
    radios_hd = [int(r * factor) for r in [8, 20, 35, 50, 65]]
    
    color_gris = (255, 255, 255)
    color_amarillo = (0, 255, 255)
    color_rastro_linea = (10, 107, 242)

    for r in radios_hd:
        cv2.circle(pantalla_negra, (centro_x_hd, centro_y_hd), r, color_gris, grosor_vector, color_aa)

    r_interno_hd = radios_hd[1]
    r_externo_hd = radios_hd[4]
    for angulo_deg in range(0, 360, 45):
        rad = np.deg2rad(angulo_deg)
        x_ini = int(centro_x_hd + r_interno_hd * np.cos(rad))
        y_ini = int(centro_y_hd + r_interno_hd * np.sin(rad))
        x_fin = int(centro_x_hd + r_externo_hd * np.cos(rad))
        y_fin = int(centro_y_hd + r_externo_hd * np.sin(rad))
        cv2.line(pantalla_negra, (x_ini, y_ini), (x_fin, y_fin), color_gris, grosor_vector, color_aa)

    angulo_rumbo_rad = np.deg2rad(datos_mpu["RumboZ"] - 90)
    x_punta_hd = int(centro_x_hd + r_externo_hd * np.cos(angulo_rumbo_rad))
    y_punta_hd = int(centro_y_hd + r_externo_hd * np.sin(angulo_rumbo_rad))
    x_cuerpo_fin_hd = int(centro_x_hd + (r_externo_hd - 1) * np.cos(angulo_rumbo_rad))
    y_cuerpo_fin_hd = int(centro_y_hd + (r_externo_hd - 1) * np.sin(angulo_rumbo_rad))

    if ult_x_cuerpo_hd is not None and (ult_x_cuerpo_hd != x_cuerpo_fin_hd or ult_y_cuerpo_hd != y_cuerpo_fin_hd):
        puntos_rastro = np.array([[centro_x_hd, centro_y_hd], [ult_x_cuerpo_hd, ult_y_cuerpo_hd], [x_cuerpo_fin_hd, y_cuerpo_fin_hd]], np.int32)
        cv2.fillPoly(capa_hud_hd, [puntos_rastro], color_rastro_linea, color_aa)

    transparent_hud_layer = capa_hud_hd
    ult_x_cuerpo_hd = x_cuerpo_fin_hd
    ult_y_cuerpo_hd = y_cuerpo_fin_hd
    pantalla_negra = cv2.add(pantalla_negra, transparent_hud_layer)

    ancho_punta_hd = int(8 * factor)
    largo_punta_hd = int(14 * factor)
    x_base_hd = centro_x_hd + (r_externo_hd - largo_punta_hd) * np.cos(angulo_rumbo_rad)
    y_base_hd = centro_y_hd + (r_externo_hd - largo_punta_hd) * np.sin(angulo_rumbo_rad)
    x_i = int(x_base_hd + ancho_punta_hd * np.cos(angulo_rumbo_rad + np.pi/2))
    y_i = int(y_base_hd + ancho_punta_hd * np.sin(angulo_rumbo_rad + np.pi/2))
    x_d = int(x_base_hd + ancho_punta_hd * np.cos(angulo_rumbo_rad - np.pi/2))
    y_d = int(y_base_hd + ancho_punta_hd * np.sin(angulo_rumbo_rad - np.pi/2))

    puntos_triangulo = np.array([[x_punta_hd, y_punta_hd], [x_i, y_i], [x_d, y_d]], np.int32)
    cv2.fillPoly(pantalla_negra, [puntos_triangulo], color_amarillo, color_aa)
    cv2.line(pantalla_negra, (centro_x_hd, centro_y_hd), (x_cuerpo_fin_hd, y_cuerpo_fin_hd), color_amarillo, max(1, int(1.5 * factor)), color_aa)

    # --- DIAL TRADICIONAL ---
    scx, scy = SCREEN_W // 2, SCREEN_H // 2
    largo_reticula_eje = int(100 * factor)
    espacio_central = int(15 * factor)

    dibujar_linea_discontinua(pantalla_negra, (scx - largo_reticula_eje, scy), (scx - espacio_central, scy), (255, 255, 255), grosor_vector, largo_segmento=int(10*factor), espacio=int(6*factor))
    dibujar_linea_discontinua(pantalla_negra, (scx + espacio_central, scy), (scx + largo_reticula_eje, scy), (255, 255, 255), grosor_vector, largo_segmento=int(10*factor), espacio=int(6*factor))
    cv2.drawMarker(pantalla_negra, (scx, scy), (255, 255, 255), cv2.MARKER_CROSS, int(16 * factor), max(1, int(1.5 * factor)), color_aa)
    
    for i in range(1, 4):
        offset_hd = int(i * 20 * factor)
        ancho_marca_hd = int(20 * factor)
        cv2.line(pantalla_negra, (scx - ancho_marca_hd, scy - offset_hd), (scx + ancho_marca_hd, scy - offset_hd), (255, 255, 255), grosor_vector, color_aa)
        cv2.line(pantalla_negra, (scx - ancho_marca_hd, scy + offset_hd), (scx + ancho_marca_hd, scy + offset_hd), (255, 255, 255), grosor_vector, color_aa)

    angulo_rad = np.deg2rad(datos_mpu["AngX"])
    sensibilidad_pitch = 2.0 * factor
    offset_y_horizonte = int(datos_mpu["AngY"] * sensibilidad_pitch)
    largo_horizonte_hd = int(300 * factor)
    
    x1_h = int(scx - largo_horizonte_hd * np.cos(angulo_rad))
    y1_h = int((scy + offset_y_horizonte) - largo_horizonte_hd * np.sin(angulo_rad))
    x2_h = int(scx + largo_horizonte_hd * np.cos(angulo_rad))
    y2_h = int((scy + offset_y_horizonte) + largo_horizonte_hd * np.sin(angulo_rad))
    cv2.line(pantalla_negra, (x1_h, y1_h), (x2_h, y2_h), (0, 0, 255), max(1, int(1.8 * factor)), color_aa)

    # Energía
    box_x_hd = (nw + x_off) - int(95 * factor)
    box_y_hd = int(45 * factor) + y_off  
    w_box_hd = int(35 * factor)
    h_box_hd = int(13 * factor)
    
    cv2.rectangle(pantalla_negra, (box_x_hd, box_y_hd), (box_x_hd + w_box_hd, box_y_hd + h_box_hd), (150, 150, 150), grosor_vector, color_aa)
    cv2.rectangle(pantalla_negra, (box_x_hd + w_box_hd, box_y_hd + int(4 * factor)), (box_x_hd + w_box_hd + int(2 * factor), box_y_hd + int(9 * factor)), (150, 150, 150), -1, color_aa)

    b1_p1 = (box_x_hd + int(3 * factor), box_y_hd + int(3 * factor))
    b1_p2 = (box_x_hd + int(11 * factor), box_y_hd + h_box_hd - int(3 * factor))
    b2_p1 = (box_x_hd + int(14 * factor), box_y_hd + int(3 * factor))
    b2_p2 = (box_x_hd + int(22 * factor), box_y_hd + h_box_hd - int(3 * factor))
    b3_p1 = (box_x_hd + int(25 * factor), box_y_hd + int(3 * factor))
    b3_p2 = (box_x_hd + w_box_hd - int(3 * factor), box_y_hd + h_box_hd - int(3 * factor))

    C_VERDE, C_AMARILLO, C_ROJO = (0, 255, 0), (0, 255, 255), (0, 0, 255)
    if 10 <= porcentaje_bateria < 25:
        cv2.rectangle(pantalla_negra, b1_p1, b1_p2, C_ROJO, -1)
    elif 25 <= porcentaje_bateria < 49:
        cv2.rectangle(pantalla_negra, b1_p1, b1_p2, C_AMARILLO, -1)
    elif 49 <= porcentaje_bateria <= 69:
        cv2.rectangle(pantalla_negra, b1_p1, b1_p2, C_VERDE, -1)
        cv2.rectangle(pantalla_negra, b2_p1, b2_p2, C_AMARILLO, -1)
    elif 69 < porcentaje_bateria < 90:
        cv2.rectangle(pantalla_negra, b1_p1, b1_p2, C_VERDE, -1)
        cv2.rectangle(pantalla_negra, b2_p1, b2_p2, C_VERDE, -1)
        cv2.rectangle(pantalla_negra, b3_p1, b3_p2, C_AMARILLO, -1)
    elif porcentaje_bateria >= 90:
        cv2.rectangle(pantalla_negra, b1_p1, b1_p2, C_VERDE, -1)
        cv2.rectangle(pantalla_negra, b2_p1, b2_p2, C_VERDE, -1)
        cv2.rectangle(pantalla_negra, b3_p1, b3_p2, C_VERDE, -1)

    # Textos HUD
    font_text = cv2.FONT_HERSHEY_SIMPLEX
    escala_sup = 0.40 * factor
    escala_telemetria = 0.50 * factor
    escala_reticula = 0.30 * factor
    grosor_txt = max(1, int(1 * factor))

    est_tiempo = time.localtime()
    txt_centro = f"{DIAS[est_tiempo.tm_wday]}, {MESES[est_tiempo.tm_mon - 1]} {est_tiempo.tm_mday}, {est_tiempo.tm_year} | {time.strftime('%H:%M:%S')}"
    txt_titulo = "SISTEMA DE VISION SUBMARINO"
    
    if estado_conexion == "LOST":
        txt_derecha = "LOST | FPS: 0"
        color_live_lost = (0, 0, 255)  
    else:
        txt_derecha = f"LIVE | FPS: {int(fps)}"
        color_live_lost = (0, 255, 0)  
        
    color_titulo = (0, 165, 255) if SIMULACION_ACTIVA else (0, 255, 0)

    (w_c, _), _ = cv2.getTextSize(txt_centro, font_text, escala_sup, grosor_txt)
    (w_d, _), _ = cv2.getTextSize(txt_derecha, font_text, escala_sup, grosor_txt)
    
    pos_y_sup = int(30 * factor) + y_off
    x_tit = int(20 * factor) + x_off
    x_fps = (nw + x_off) - w_d - int(20 * factor)
    x_cen = x_fps - w_c - int(30 * factor)

    texto_con_borde(pantalla_negra, txt_titulo, (x_tit, pos_y_sup), font_text, escala_sup, color_titulo, grosor_txt)
    texto_con_borde(pantalla_negra, txt_centro, (x_cen, pos_y_sup), font_text, escala_sup, (255, 255, 255), grosor_txt)
    texto_con_borde(pantalla_negra, txt_derecha, (x_fps, pos_y_sup), font_text, escala_sup, color_live_lost, grosor_txt)

    if grabando:
        opacidad_rec = (math.sin(t_actual * 7) + 1) / 2 
        if opacidad_rec > 0.25:  
            txt_rec = "REC"
            (w_rec, _), _ = cv2.getTextSize(txt_rec, font_text, escala_sup, grosor_txt)
            x_rec = x_cen - w_rec - int(15 * factor)
            texto_con_borde(pantalla_negra, txt_rec, (x_rec, pos_y_sup), font_text, escala_sup, (0, 0, 255), grosor_txt)

    x_valores_energia = (nw + x_off) - int(50 * factor)
    y_bat = int(56 * factor) + y_off
    y_vol = int(75 * factor) + y_off
    y_cor = int(94 * factor) + y_off

    texto_con_borde(pantalla_negra, f"{porcentaje_bateria}%", (x_valores_energia, y_bat), font_text, escala_sup, (255, 255, 255), grosor_txt)
    texto_con_borde(pantalla_negra, f"{voltaje_bateria:.2f} V", (x_valores_energia - int(45 * factor), y_vol), font_text, escala_sup, (255, 255, 255), grosor_txt)
    texto_con_borde(pantalla_negra, f"{corriente_mA:.1f} mA", (x_valores_energia - int(45 * factor), y_cor), font_text, escala_sup, (255, 255, 255), grosor_txt)

    for i in range(1, 4):
        offset_hd = int(i * 20 * factor)
        texto_con_borde(pantalla_negra, str(i*10), (scx + int(25*factor), scy - offset_hd + int(3*factor)), font_text, escala_reticula, (255, 255, 255), grosor_txt)
        texto_con_borde(pantalla_negra, str(i*10), (scx + int(25*factor), scy + offset_hd + int(3*factor)), font_text, escala_reticula, (255, 255, 255), grosor_txt)

    texto_con_borde(pantalla_negra, f"PITCH: {datos_mpu['AngY']:.1f}", (scx + int(140*factor), scy - int(10*factor)), font_text, escala_telemetria, (255, 255, 255), grosor_txt)
    texto_con_borde(pantalla_negra, f"ROLL: {datos_mpu['AngX']:.1f}", (scx + int(140*factor), scy + int(10*factor)), font_text, escala_telemetria, (255, 255, 255), grosor_txt)
    
    txt_rumbo_valor = f"{datos_mpu['RumboZ']:.1f}"
    (w_r, _), _ = cv2.getTextSize(txt_rumbo_valor, font_text, escala_telemetria, grosor_txt)
    texto_con_borde(pantalla_negra, txt_rumbo_valor, (centro_x_hd - (w_r // 2), centro_y_hd + radios_hd[4] + int(25 * factor)), font_text, escala_telemetria, (255, 255, 255), grosor_txt)

    escala_inferior = 0.46 * factor  
    txt_temp = f"TEMP: {temperatura_c:.1f} C"
    txt_prof = f"PROF: {profundidad_m:.2f} Mts"
    txt_presion = f"PRESION: {presion_bar:.3f} bar"
    
    pos_x_inf_izq = int(20 * factor) + x_off
    pos_y_bar = (nh + y_off) - int(25 * factor)              
    pos_y_prof = pos_y_bar - int(22 * factor)                
    pos_y_temp = pos_y_prof - int(22 * factor)                

    texto_con_borde(pantalla_negra, txt_temp, (pos_x_inf_izq, pos_y_temp), font_text, escala_inferior, (255, 255, 255), grosor_txt)
    texto_con_borde(pantalla_negra, txt_prof, (pos_x_inf_izq, pos_y_prof), font_text, escala_inferior, (255, 255, 255), grosor_txt)
    texto_con_borde(pantalla_negra, txt_presion, (pos_x_inf_izq, pos_y_bar), font_text, escala_inferior, (255, 255, 255), grosor_txt)

    txt_mode = f"MODE: {modo_sistema}"
    txt_ping = f"PING: {ping_ms} ms"
    txt_time = f"TIME: {formatear_segundos(tiempo_run_total)}"

    (w_mode, _), _ = cv2.getTextSize(txt_mode, font_text, escala_inferior, grosor_txt)
    (w_ping, _), _ = cv2.getTextSize(txt_ping, font_text, escala_inferior, grosor_txt)
    (w_time, _), _ = cv2.getTextSize(txt_time, font_text, escala_inferior, grosor_txt)

    pos_x_mode = (nw + x_off) - w_mode - int(20 * factor)
    pos_x_ping = (nw + x_off) - w_ping - int(20 * factor)
    pos_x_time = (nw + x_off) - w_time - int(20 * factor)

    pos_y_time_der = (nh + y_off) - int(25 * factor)          
    pos_y_ping_der = pos_y_time_der - int(22 * factor)         
    pos_y_mode_der = pos_y_ping_der - int(22 * factor)         

    texto_con_borde(pantalla_negra, txt_mode, (pos_x_mode, pos_y_mode_der), font_text, escala_inferior, (255, 255, 255), grosor_txt)
    texto_con_borde(pantalla_negra, txt_ping, (pos_x_ping, pos_y_ping_der), font_text, escala_inferior, (255, 255, 255), grosor_txt)
    texto_con_borde(pantalla_negra, txt_time, (pos_x_time, pos_y_time_der), font_text, escala_inferior, (255, 255, 255), grosor_txt)

    # --- INDICADOR DEL MANDO CENTRADO ABAJO (SOLO SI SE PRESIONA ALGO) ---
    if hubo_actividad and boton_actual_texto != "":
        txt_mando_debug = f"INPUT: {boton_actual_texto}"
        color_mando_txt = (0, 255, 255) 
        
        (w_mando, _), _ = cv2.getTextSize(txt_mando_debug, font_text, escala_inferior * 1.1, grosor_txt)
        pos_x_mando_centrado = (SCREEN_W - w_mando) // 2
        pos_y_mando_centrado = pos_y_mode_der - int(10 * factor) 
        
        texto_con_borde(pantalla_negra, txt_mando_debug, (pos_x_mando_centrado, pos_y_mando_centrado), font_text, escala_inferior * 1.1, color_mando_txt, grosor_txt)

    if grabando and video_writer is not None:
        video_writer.write(pantalla_negra)

    cv2.imshow(nombre_ventana, pantalla_negra)
    
    tecla = cv2.waitKey(1) & 0xFF
    if tecla == ord('q') or tecla == ord('Q'):
        ejecutando = False
    elif tecla == ord('r') or tecla == ord('R'):
        conmutar_grabacion()

# ==========================================================
# CIERRE SEGURO
# ==========================================================
print("\nApagando sistemas...")
if video_writer is not None:
    video_writer.release()
if ser: 
    try: ser.close()
    except: pass
cap.release()
cv2.destroyAllWindows()
pygame.quit()
print(f"HUD Cerrado. Las grabaciones se guardaron en:\n📂 {RUTA_ABSOLUTA}")