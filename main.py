import matplotlib.pyplot as plt
import json

# Dimensiones del mapa
map_width = 6
map_height = 4

# Posiciones iniciales de los drones
drone_positions = [(-1.5, -2), (-2, 0)]
drone_size = 0.2
drone_colors = ['green', 'red']

# Obstáculos (cada obstáculo se define por sus 4 esquinas)
obstacles = [
    [(-0.218, 0.226), (-0.746, -0.079), (-0.442, -0.607), (0.086, -0.302)],
    [(0.442, 0.607), (-0.086, 0.302), (0.218, -0.226), (0.746, 0.079)],
    [(-0.6952, 1.3048), (-1.3048, 1.3048), (-1.3048, 0.6952), (-0.6952, 0.6952)],
    [(1.3048, -0.6952), (0.6952, -0.6952), (0.6952, -1.3048), (1.3048, -1.3048)],
    [(0.8733, 1.4355), (0.2845, 1.5933), (0.1267, 1.0045), (0.7155, 0.8467)],
    [(-0.5689, -1), (-1, -0.5689), (-1.431, -1), (-1, -1.431)]
]

# Puntos objetivo solo para visualización
target_positions = [
    (-0.867, -0.357), (-0.277, 0.550), (0.286, -0.497), (-1.017, 1.527), 
    (0.673, 0.629), (-1.378, -1.369), (1.545, -0.999)
]

# Lista para almacenar los puntos de la ruta
drone_paths = {"drone_1": [], "drone_2": []}
current_drone = 0

def onclick(event):
    """Registra los puntos seleccionados y los muestra en el gráfico."""
    if event.xdata is not None and event.ydata is not None:
        drone_key = "drone_1" if current_drone == 0 else "drone_2"
        drone_paths[drone_key].append((event.xdata, event.ydata))
        ax.plot(event.xdata, event.ydata, 'go' if current_drone == 0 else 'bo')  # Punto verde o azul
        plt.draw()

def on_key(event):
    """Cambia el dron activo al presionar Enter."""
    global current_drone
    if event.key == 'enter':
        current_drone = 1 - current_drone
        print(f"Cambiando a { 'drone_1' if current_drone == 0 else 'drone_2' }")

def save_path():
    """Guarda los puntos en un archivo JSON."""
    with open("TargetPositions.txt", "w") as file:
        json.dump(drone_paths, file, indent=4)
    print("Ruta guardada en TargetPositions.txt")

# Configurar el mapa
fig, ax = plt.subplots()
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
ax.set_xlabel("X (metros)")
ax.set_ylabel("Y (metros)")
ax.set_title("Simulación de Ruta del Dron")
ax.grid(True)

# Dibujar obstáculos
for obs in obstacles:
    x_coords, y_coords = zip(*obs)
    ax.fill(x_coords, y_coords, color='blue', alpha=0.6)

# Dibujar drones
for i, (x, y) in enumerate(drone_positions):
    ax.add_patch(plt.Rectangle((x - drone_size/2, y - drone_size/2), drone_size, drone_size, color=drone_colors[i]))

# Dibujar puntos objetivo
for x, y in target_positions:
    ax.plot(x, y, 'ro')

# Conectar eventos de clic y teclado
fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('key_press_event', on_key)

# Mostrar la ventana
plt.show()

# Guardar los puntos cuando se cierre la ventana
save_path()