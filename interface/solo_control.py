"""Controla el submarino con el mando PG-9076 y el ESP32 por puerto serie."""

import os
import sys
import time

# El programa usa una ventana oculta; permite que SDL actualice el mando sin foco.
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame
import serial


# Cambiar COM3 si Windows asigna otro puerto al ESP32.
PUERTO_COM = "COM3"
BAUDIOS = 115200
DEADZONE = 0.25
REENVIO_MOVIMIENTO_S = 0.15
ESPERA_ESP32_S = 30

# Mapeo medido en prueba control.py; Pygame entrega algunos ejes duplicados.
EJE_IZQUIERDO_Y = 1
# EJE[2] duplica al motor; EJE[5] duplica el vertical derecho.
EJE_DERECHO_X = 3
EJE_DERECHO_Y = 4

# Indices confirmados en la prueba de Windows/Pygame.
BOTON_A, BOTON_B, BOTON_X, BOTON_Y = 0, 1, 3, 4
BOTON_START = 11

# Debe coincidir con ESC_REVERSIBLE en el sketch Arduino.
ESC_REVERSIBLE = True
PULSO_ESC_NEUTRO = 1500 if ESC_REVERSIBLE else 1000


def limitar(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


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
        print(f"> {comando}")
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


def actualizar_valvula():
    enviar("valvula_on" if aire_1_activo or aire_2_activo else "valvula_off")


def alternar_aire(numero):
    global aire_1_activo, aire_2_activo
    if numero == 1:
        aire_1_activo = not aire_1_activo
        enviar("aire1_on" if aire_1_activo else "aire1_off")
    else:
        aire_2_activo = not aire_2_activo
        enviar("aire2_on" if aire_2_activo else "aire2_off")
    actualizar_valvula()


print("Conectado. Palanca izquierda: ESC; derecha: bombas de agua.")
print("X/Y: bombas de aire; A/B: abrir/cerrar pinza; Start: stop.")

reloj = pygame.time.Clock()
ejecutando = True
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
                if evento.button == BOTON_X:
                    alternar_aire(1)
                elif evento.button == BOTON_Y:
                    alternar_aire(2)
                elif evento.button == BOTON_A:
                    enviar("abrir")
                elif evento.button == BOTON_B:
                    enviar("cerrar")
                elif evento.button == BOTON_START:
                    aire_1_activo = aire_2_activo = False
                    enviar("stop")
                    ultimo_comando["esc"] = ultimo_comando["agua"] = None
                    bloqueo_hasta_reposo = True

        if ejecutando:
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
                print(f"ESP32: {respuesta}")

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
    pygame.quit()
