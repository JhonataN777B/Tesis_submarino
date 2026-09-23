"""Diagnostico del mando: muestra indices de ejes, botones y cruceta."""

import os
import sys

# El programa usa una ventana oculta; permite que SDL actualice el mando sin foco.
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame


pygame.init()
pygame.display.set_mode((1, 1), pygame.HIDDEN)

if pygame.joystick.get_count() == 0:
    print("No se detecto un mando Bluetooth. Conectelo y vuelva a ejecutar.")
    pygame.quit()
    sys.exit(1)

joystick = pygame.joystick.Joystick(0)
print(f"Mando detectado: {joystick.get_name()}")
print(f"Ejes: {joystick.get_numaxes()} | Botones: {joystick.get_numbuttons()} | Crucetas: {joystick.get_numhats()}")
print("Mueva una palanca o trigger; cada cambio aparecera en una linea nueva.")
print("Pulse cada boton y mueva la cruceta. Ctrl+C para terminar.\n")

reloj = pygame.time.Clock()
ejecutando = True
ejes_anteriores = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
botones_anteriores = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
crucetas_anteriores = [joystick.get_hat(i) for i in range(joystick.get_numhats())]

print("Estado inicial:")
print("  Ejes: " + " ".join(f"{i}={valor:+.2f}" for i, valor in enumerate(ejes_anteriores)))
print("  Botones presionados: " + str([i for i, activo in enumerate(botones_anteriores) if activo]))
print("  Crucetas: " + str(crucetas_anteriores))

try:
    while ejecutando:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                ejecutando = False
            elif evento.type == pygame.JOYDEVICEREMOVED:
                print("\nSe desconecto el mando.")
                ejecutando = False

        if not ejecutando:
            break

        pygame.event.pump()
        ejes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        botones = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        crucetas = [joystick.get_hat(i) for i in range(joystick.get_numhats())]

        for i, (anterior, actual) in enumerate(zip(ejes_anteriores, ejes)):
            if abs(actual - anterior) >= 0.02:
                print(f"EJE[{i}]: {anterior:+.2f} -> {actual:+.2f}", flush=True)
        for i, (anterior, actual) in enumerate(zip(botones_anteriores, botones)):
            if actual != anterior:
                estado = "presionado" if actual else "liberado"
                print(f"BOTON[{i}] {estado}", flush=True)
        for i, (anterior, actual) in enumerate(zip(crucetas_anteriores, crucetas)):
            if actual != anterior:
                print(f"CRUCETA[{i}]: {anterior} -> {actual}", flush=True)

        ejes_anteriores = ejes
        botones_anteriores = botones
        crucetas_anteriores = crucetas
        reloj.tick(10)
except KeyboardInterrupt:
    print("\nPrueba del mando finalizada.")
finally:
    pygame.quit()
