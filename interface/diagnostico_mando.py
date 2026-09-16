import pygame
import sys

# Inicializar Pygame y el módulo de joysticks
pygame.init()
pygame.joystick.init()

# Verificar si hay un control conectado
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No se detectó ningún control de Xbox. ¡Conéctalo e intenta de nuevo!")
    sys.exit()

# Seleccionar el primer control disponible
xbox_controller = pygame.joystick.Joystick(0)
xbox_controller.init()

print(f"--- Control detectado: {xbox_controller.get_name()} ---")
print("Leyendo datos... Presiona Ctrl+C en la terminal para salir.\n")

# TRUCO: Creamos una ventana de 1x1 píxeles y la ocultamos con el flag HIDDEN
# Así el sistema operativo envía los datos del control, pero tú no ves nada.
screen = pygame.display.set_mode((1, 1), pygame.HIDDEN)

# Diccionario para rastrear qué botones ya están presionados (evita spam en la consola)
botones_presionados = {}

clock = pygame.time.Clock()

try:
    while True:
        # Procesar los eventos de Pygame
        for event in pygame.event.get():
            # 1. DETECTAR BOTONES (Empezando desde el 1)
            if event.type == pygame.JOYBUTTONDOWN:
                boton_natural = event.button + 1
                print(f"[BOTÓN] Presionado: Botón {boton_natural}")
                
            elif event.type == pygame.JOYBUTTONUP:
                boton_natural = event.button + 1
                print(f"[BOTÓN] Soltado: Botón {boton_natural}")
                
            # 2. DETECTAR LA CRUCETA / D-PAD
            elif event.type == pygame.JOYHATMOTION:
                if event.value != (0, 0):
                    print(f"[CRUCETA] Posición: {event.value}")

        # 3. DETECTAR PALANCAS Y GATILLOS ANALÓGICOS (Empezando desde el 1)
        num_axes = xbox_controller.get_numaxes()
        for i in range(num_axes):
            axis_value = xbox_controller.get_axis(i)
            if abs(axis_value) > 0.2:  # Zona muerta para evitar lecturas fantasmas
                print(f"[EJE] Eje {i + 1} movido a: {axis_value:.2f}")

        # Mantener el bucle estable a 60 actualizaciones por segundo
        clock.tick(60)

except KeyboardInterrupt:
    print("\nLectura finalizada por el usuario.")
finally:
    pygame.quit()