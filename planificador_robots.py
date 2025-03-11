import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from shapely.geometry import Point, Polygon as ShapelyPolygon, LineString
import networkx as nx
import heapq
import math
from itertools import permutations

# Posiciones iniciales
positions = [(-1.5, -2), (-2, 0)]
size = 0.2
colors = ['green', 'red']

# Obstáculos (cada obstáculo se define por sus 4 esquinas)
obstacles = [
    [(-0.218, 0.226), (-0.746, -0.079), (-0.442, -0.607), (0.086, -0.302)],
    [(0.442, 0.607), (-0.086, 0.302), (0.218, -0.226), (0.746, 0.079)],
    [(-0.6952, 1.3048), (-1.3048, 1.3048), (-1.3048, 0.6952), (-0.6952, 0.6952)],
    [(1.3048, -0.6952), (0.6952, -0.6952), (0.6952, -1.3048), (1.3048, -1.3048)],
    [(0.8733, 1.4355), (0.2845, 1.5933), (0.1267, 1.0045), (0.7155, 0.8467)],
    [(-0.5689, -1), (-1, -0.5689), (-1.431, -1), (-1, -1.431)]
]

# Puntos objetivo
target_positions = [
    (-0.86717069892473, -0.356552419354838),
    (-0.277318548387096, 0.550235215053764),
    (0.286122311827957, -0.497412634408602),
    (-1.01683467741935, 1.52745295698925),
    (0.673487903225808, 0.629469086021506),
    (-1.37778897849462, -1.36898521505376),
    (1.54506048387097, -0.999227150537633)
]

# Lista para almacenar los puntos de la ruta
robots = {"robot1": [], "robot2": []}

# Convertir obstáculos a polígonos de Shapely
shapely_obstacles = [ShapelyPolygon(obs) for obs in obstacles]

# Función para verificar si un punto está en colisión con obstáculos
def is_collision(point, obstacles, buffer=size):
    p = Point(point)
    buffered_point = p.buffer(buffer)
    
    for obs in obstacles:
        if buffered_point.intersects(obs):
            return True
    return False

# Función para verificar si hay colisión entre robots
def is_robot_collision(pos1, pos2, buffer=size*2):
    return Point(pos1).distance(Point(pos2)) < buffer

# Función para verificar si una línea interseca obstáculos
def line_collision(p1, p2, obstacles, buffer=size):
    line = LineString([p1, p2])
    buffered_line = line.buffer(buffer)
    
    for obs in obstacles:
        if buffered_line.intersects(obs):
            return True
    return False

# Crear un grafo de navegación para A*
def create_navigation_graph(obstacles, x_min=-2.5, x_max=2.5, y_min=-2.5, y_max=2.5, resolution=0.1):
    G = nx.Graph()
    
    # Crear nodos en una cuadrícula
    for x in np.arange(x_min, x_max, resolution):
        for y in np.arange(y_min, y_max, resolution):
            point = (round(x, 2), round(y, 2))
            if not is_collision(point, obstacles):
                G.add_node(point)
    
    # Conectar nodos adyacentes
    for node in G.nodes():
        x, y = node
        for dx, dy in [(resolution, 0), (-resolution, 0), (0, resolution), (0, -resolution),
                       (resolution, resolution), (-resolution, resolution), 
                       (resolution, -resolution), (-resolution, -resolution)]:
            neighbor = (round(x + dx, 2), round(y + dy, 2))
            if neighbor in G and not line_collision(node, neighbor, obstacles):
                # Usar distancia euclidiana como peso
                weight = math.sqrt(dx**2 + dy**2)
                G.add_edge(node, neighbor, weight=weight)
    
    return G

# Implementación del algoritmo A*
def astar(graph, start, goal, obstacles, other_robot_path=None):
    if start not in graph or goal not in graph:
        # Encontrar nodos más cercanos en el grafo
        start_node = min(graph.nodes(), key=lambda n: Point(start).distance(Point(n)))
        goal_node = min(graph.nodes(), key=lambda n: Point(goal).distance(Point(n)))
    else:
        start_node, goal_node = start, goal
    
    # Cola de prioridad para A*
    frontier = [(0, start_node)]
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}
    
    while frontier:
        _, current = heapq.heappop(frontier)
        
        if current == goal_node:
            break
        
        for next_node in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph[current][next_node]['weight']
            
            # Verificar colisión temporal con otro robot si hay una ruta
            if other_robot_path:
                # Si hay otro robot en ese punto en el mismo paso, evitarlo
                time_step = int(new_cost / 0.1)  # Aproximación de tiempo basada en distancia
                if time_step < len(other_robot_path):
                    other_pos = other_robot_path[time_step]
                    if is_robot_collision(next_node, other_pos):
                        continue
            
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                # Heurística: distancia euclidiana
                priority = new_cost + math.sqrt((next_node[0] - goal_node[0])**2 + 
                                               (next_node[1] - goal_node[1])**2)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
    
    # Reconstruir el camino
    path = []
    current = goal_node
    if current in came_from:
        while current:
            path.append(current)
            current = came_from[current]
        path.reverse()
    
    return path

# Modificación de la función para generar rutas por todos los targets
def find_optimal_path_order(start_pos, targets, obstacles):
    """Encuentra el orden óptimo para visitar todos los targets desde una posición inicial"""
    best_order = None
    best_distance = float('inf')
    
    # Probar todas las permutaciones posibles
    for perm in permutations(targets):
        total_dist = Point(start_pos).distance(Point(perm[0]))
        for i in range(len(perm) - 1):
            total_dist += Point(perm[i]).distance(Point(perm[i + 1]))
        
        if total_dist < best_distance:
            best_distance = total_dist
            best_order = perm
    
    return list(best_order)

# Modificar la planificación para asegurar que los robots terminen en puntos diferentes
print("Calculando orden óptimo para robot 1...")
robot1_targets_order = find_optimal_path_order(positions[0], target_positions, shapely_obstacles)
ultimo_target_robot1 = robot1_targets_order[-1]  # Guardar el último objetivo del robot 1

print("Calculando orden óptimo para robot 2 (con punto final diferente)...")
# Modificamos la función para forzar un punto final distinto para el robot 2
def find_optimal_path_order_different_end(start_pos, targets, last_point_to_avoid, obstacles):
    """Encuentra el orden óptimo para visitar todos los targets con un punto final diferente"""
    best_order = None
    best_distance = float('inf')
    
    for perm in permutations(targets):
        # Verificar que el último punto de la permutación no sea el mismo que el último del robot 1
        if perm[-1] == last_point_to_avoid:
            continue
            
        total_dist = Point(start_pos).distance(Point(perm[0]))
        for i in range(len(perm) - 1):
            total_dist += Point(perm[i]).distance(Point(perm[i + 1]))
        
        if total_dist < best_distance:
            best_distance = total_dist
            best_order = perm
    
    return list(best_order)

# Usar la nueva función para el robot 2
robot2_targets_order = find_optimal_path_order_different_end(
    positions[1], target_positions, ultimo_target_robot1, shapely_obstacles)

# Crear grafo de navegación (restaurado)
print("Creando grafo de navegación...")
nav_graph = create_navigation_graph(shapely_obstacles)
print(f"Grafo creado con {len(nav_graph.nodes())} nodos y {len(nav_graph.edges())} aristas")

# Planificación de rutas para el robot 1
print("Planificando ruta para robot 1...")
robot1_path = [positions[0]]
current_pos = positions[0]

for target in robot1_targets_order:
    path = astar(nav_graph, current_pos, target, shapely_obstacles)
    if path:
        # Agregar el camino (omitiendo el punto inicial que ya está en la ruta)
        robot1_path.extend(path[1:])
        current_pos = target
    else:
        print(f"No se pudo encontrar ruta al objetivo {target} para robot 1")

# Mejorar el manejo de colisiones para el robot 2 con puntos de desvío
print("Planificando ruta para robot 2...")
robot2_path = [positions[1]]
current_pos = positions[1]

# Añadir función para encontrar punto de desvío
def find_detour_point(current_pos, target_pos, robot1_path, obstacles, radius=0.4):
    """Encuentra un punto de desvío seguro para evitar colisión con robot 1"""
    # Vector desde posición actual al objetivo
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    
    # Normalizar el vector
    length = math.sqrt(dx**2 + dy**2)
    if length > 0:
        dx, dy = dx/length, dy/length
    
    # Probar diferentes ángulos para encontrar un punto de desvío
    best_point = None
    best_score = float('inf')
    
    for angle_offset in range(0, 360, 45):  # Probar cada 45 grados
        rad = math.radians(angle_offset)
        # Rotar el vector
        new_dx = dx * math.cos(rad) - dy * math.sin(rad)
        new_dy = dx * math.sin(rad) + dy * math.cos(rad)
        
        # Crear punto de desvío
        detour_point = (current_pos[0] + radius * new_dx, 
                         current_pos[1] + radius * new_dy)
        
        # Verificar si el punto está libre de obstáculos
        if not is_collision(detour_point, obstacles):
            # Calcular distancia mínima a la ruta del robot 1
            min_dist = float('inf')
            for pos in robot1_path:
                dist = Point(detour_point).distance(Point(pos))
                min_dist = min(min_dist, dist)
            
            # Puntuación: combinación de distancia al robot 1 y distancia al objetivo
            # Preferimos puntos que estén lejos del robot 1 pero no se alejen mucho del objetivo
            dist_to_target = Point(detour_point).distance(Point(target_pos))
            score = -min_dist + 0.5 * dist_to_target
            
            if score < best_score:
                best_score = score
                best_point = detour_point
    
    return best_point

for target in robot2_targets_order:
    # Intentar encontrar ruta directa
    path = astar(nav_graph, current_pos, target, shapely_obstacles, robot1_path)
    
    if path:
        # Verificar colisiones en la ruta y usar desvíos si es necesario
        temp_path = []
        robot2_pos_idx = len(robot2_path)
        i = 1  # Empezar desde el segundo punto (el primero ya está en la ruta)
        
        while i < len(path):
            pos = path[i]
            current_time_step = robot2_pos_idx + len(temp_path)
            
            # Verificar colisión con robot 1
            collision_risk = False
            for t_offset in range(-2, 3):  # Ventana más amplia para detección
                time_idx = current_time_step + t_offset
                if 0 <= time_idx < len(robot1_path):
                    robot1_pos = robot1_path[time_idx]
                    if is_robot_collision(pos, robot1_pos):
                        collision_risk = True
                        break
            
            if collision_risk:
                # Calcular punto de desvío
                last_pos = temp_path[-1] if temp_path else current_pos
                detour_point = find_detour_point(last_pos, target, robot1_path, shapely_obstacles)
                
                if detour_point:
                    # Calcular ruta al punto de desvío
                    detour_path = astar(nav_graph, last_pos, detour_point, shapely_obstacles)
                    if detour_path and len(detour_path) > 1:
                        temp_path.extend(detour_path[1:])
                        
                        # Recalcular ruta desde el punto de desvío al objetivo
                        new_path = astar(nav_graph, detour_point, target, shapely_obstacles, robot1_path)
                        if new_path and len(new_path) > 1:
                            # Reiniciar el proceso con la nueva ruta
                            path = new_path
                            i = 1
                            continue
                
                # Si no se puede calcular desvío, añadir espera
                for _ in range(3):  # Esperar 3 pasos
                    if temp_path:
                        temp_path.append(temp_path[-1])
                    else:
                        temp_path.append(current_pos)
            
            # Añadir el punto actual y continuar
            temp_path.append(pos)
            i += 1
        
        # Actualizar el camino y la posición
        robot2_path.extend(temp_path)
        current_pos = target
        
    else:
        print(f"No se pudo encontrar ruta directa al objetivo {target} para robot 2")
        
        # Generar puntos de desvío aleatorios y tratar de llegar al objetivo a través de ellos
        for _ in range(5):  # Intentar hasta 5 puntos de desvío diferentes
            # Generar un punto de desvío en una dirección aleatoria
            angle = np.random.uniform(0, 2*np.pi)
            distance = np.random.uniform(0.5, 1.0)
            detour_x = current_pos[0] + distance * np.cos(angle)
            detour_y = current_pos[1] + distance * np.sin(angle)
            detour_point = (detour_x, detour_y)
            
            # Verificar que el punto esté dentro de los límites y libre de obstáculos
            if -2.5 <= detour_x <= 2.5 and -2.5 <= detour_y <= 2.5 and not is_collision(detour_point, shapely_obstacles):
                # Intentar llegar al punto de desvío
                detour_path = astar(nav_graph, current_pos, detour_point, shapely_obstacles, robot1_path)
                if detour_path and len(detour_path) > 1:
                    # Luego intentar desde el desvío al objetivo
                    remaining_path = astar(nav_graph, detour_point, target, shapely_obstacles, robot1_path)
                    if remaining_path and len(remaining_path) > 1:
                        # Añadir el camino completo
                        robot2_path.extend(detour_path[1:])
                        robot2_path.extend(remaining_path[1:])
                        current_pos = target
                        break
        
        # Si después de todos los intentos no se puede llegar al objetivo
        if current_pos != target:
            print(f"No se pudo encontrar ruta alternativa para robot 2 al objetivo {target}")
            # Como último recurso, esperar y luego intentar ir directo ignorando colisiones con robot 1
            for _ in range(10):
                robot2_path.append(current_pos)
            direct_path = astar(nav_graph, current_pos, target, shapely_obstacles)
            if direct_path:
                robot2_path.extend(direct_path[1:])
                current_pos = target
            else:
                print(f"Imposible llegar al objetivo {target} para robot 2")

# Guardar rutas
robots["robot1"] = robot1_path
robots["robot2"] = robot2_path

# Visualización
plt.figure(figsize=(10, 10))
ax = plt.gca()

# Dibujar obstáculos
for obs in obstacles:
    ax.add_patch(Polygon(obs, closed=True, fill=True, color='gray', alpha=0.5))

# Dibujar puntos objetivo
for pos in target_positions:
    ax.add_patch(Circle(pos, radius=0.05, color='blue'))

# Dibujar rutas
plt.plot([p[0] for p in robots["robot1"]], [p[1] for p in robots["robot1"]], 'g-', linewidth=2)
plt.plot([p[0] for p in robots["robot2"]], [p[1] for p in robots["robot2"]], 'r-', linewidth=2)

# Dibujar posiciones iniciales
plt.scatter([pos[0] for pos in positions], [pos[1] for pos in positions], 
            c=colors, s=100, zorder=10)

# Configuración del gráfico
plt.grid(True)
plt.axis('equal')
plt.xlim(-2.5, 2.5)
plt.ylim(-2.5, 2.5)
plt.title('Planificación de rutas para dos robots')
plt.savefig('rutas_robots.png')
plt.show()

# Imprimir las rutas calculadas
print("\nRuta del Robot 1:")
for i, point in enumerate(robots["robot1"]):
    print(f"Punto {i}: {point}")

print("\nRuta del Robot 2:")
for i, point in enumerate(robots["robot2"]):
    print(f"Punto {i}: {point}")

# Añadir información sobre puntos finales al guardar los archivos y en la visualización
print("\nPunto final del Robot 1:", robots["robot1"][-1])
print("Punto final del Robot 2:", robots["robot2"][-1])

# Al guardar las rutas, añadir información sobre los puntos finales
with open("ruta_robot1.txt", "w") as f:
    f.write(f"# Punto inicial: {positions[0]}\n")
    f.write(f"# Punto final: {robots['robot1'][-1]}\n")
    f.write(f"# Total de puntos: {len(robots['robot1'])}\n")
    for point in robots["robot1"]:
        f.write(f"{point[0]},{point[1]}\n")
    print("Ruta del robot 1 guardada en 'ruta_robot1.txt'")

with open("ruta_robot2.txt", "w") as f:
    f.write(f"# Punto inicial: {positions[1]}\n")
    f.write(f"# Punto final: {robots['robot2'][-1]}\n")
    f.write(f"# Total de puntos: {len(robots['robot2'])}\n")
    for point in robots["robot2"]:
        f.write(f"{point[0]},{point[1]}\n")
    print("Ruta del robot 2 guardada en 'ruta_robot2.txt'")

# Simulación del movimiento de los robots
print("\nCreando simulación del movimiento de los robots...")

# Crear una nueva figura para la animación
fig_sim, ax_sim = plt.subplots(figsize=(10, 10))

# Dibujar obstáculos
for obs in obstacles:
    ax_sim.add_patch(Polygon(obs, closed=True, fill=True, color='gray', alpha=0.5))

# Dibujar puntos objetivo
target_circles = []
for pos in target_positions:
    circle = ax_sim.add_patch(Circle(pos, radius=0.05, color='blue', alpha=0.7))
    target_circles.append(circle)

# Determinar la longitud máxima de las rutas para la animación
max_length = max(len(robots["robot1"]), len(robots["robot2"]))

# Crear marcadores para los robots
robot_markers = []
for i in range(2):
    marker, = ax_sim.plot([], [], 'o', color=colors[i], markersize=10, zorder=10)
    robot_markers.append(marker)

# Crear líneas para las rutas recorridas
robot_paths = []
for i in range(2):
    path, = ax_sim.plot([], [], '-', color=colors[i], linewidth=2, alpha=0.6)
    robot_paths.append(path)

# Texto para mostrar el tiempo
time_text = ax_sim.text(0.02, 0.98, '', transform=ax_sim.transAxes, fontsize=12, 
                         verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))

# Texto para mostrar información sobre objetivos alcanzados
target_text = ax_sim.text(0.02, 0.94, '', transform=ax_sim.transAxes, fontsize=12,
                         verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))

# Lista para llevar registro de los objetivos alcanzados
targets_reached = [0, 0]  # [robot1, robot2]
visited_targets = [set(), set()]  # Conjuntos para los objetivos visitados por cada robot

# Función de inicialización de la animación
def init_animation():
    for marker in robot_markers:
        marker.set_data([], [])
    for path in robot_paths:
        path.set_data([], [])
    time_text.set_text('')
    target_text.set_text('')
    return robot_markers + robot_paths + [time_text, target_text]

# Función de actualización para cada frame
def update_animation(frame):
    robot_positions = []
    
    # Actualizar posición del robot 1
    if frame < len(robots["robot1"]):
        pos1 = robots["robot1"][frame]
        robot_positions.append(pos1)
        robot_markers[0].set_data([pos1[0]], [pos1[1]])
        
        # Actualizar ruta recorrida
        x_data = [robots["robot1"][i][0] for i in range(frame + 1)]
        y_data = [robots["robot1"][i][1] for i in range(frame + 1)]
        robot_paths[0].set_data(x_data, y_data)
        
        # Verificar si se alcanzó un objetivo
        for i, target in enumerate(target_positions):
            if Point(pos1).distance(Point(target)) < 0.1 and i not in visited_targets[0]:
                visited_targets[0].add(i)
                targets_reached[0] += 1
                # Marcar el objetivo como visitado cambiando su color
                target_circles[i].set_facecolor('lightgreen')
    else:
        # Si el robot ya completó su ruta, mantener su última posición
        if robots["robot1"]:
            pos1 = robots["robot1"][-1]
            robot_positions.append(pos1)
            robot_markers[0].set_data([pos1[0]], [pos1[1]])
    
    # Actualizar posición del robot 2
    if frame < len(robots["robot2"]):
        pos2 = robots["robot2"][frame]
        robot_positions.append(pos2)
        robot_markers[1].set_data([pos2[0]], [pos2[1]])
        
        # Actualizar ruta recorrida
        x_data = [robots["robot2"][i][0] for i in range(frame + 1)]
        y_data = [robots["robot2"][i][1] for i in range(frame + 1)]
        robot_paths[1].set_data(x_data, y_data)
        
        # Verificar si se alcanzó un objetivo
        for i, target in enumerate(target_positions):
            if Point(pos2).distance(Point(target)) < 0.1 and i not in visited_targets[1]:
                visited_targets[1].add(i)
                targets_reached[1] += 1
                # Marcar el objetivo como visitado (si no fue visitado por robot 1)
                if target_circles[i].get_facecolor()[0] != 0.8:  # No es verde claro
                    target_circles[i].set_facecolor('salmon')
    else:
        # Si el robot ya completó su ruta, mantener su última posición
        if robots["robot2"]:
            pos2 = robots["robot2"][-1]
            robot_positions.append(pos2)
            robot_markers[1].set_data([pos2[0]], [pos2[1]])
    
    # Verificar colisión entre robots
    if len(robot_positions) == 2:
        if Point(robot_positions[0]).distance(Point(robot_positions[1])) < size*2:
            # Marcar colisión con color rojo más intenso
            robot_markers[0].set_markerfacecolor('darkred')
            robot_markers[1].set_markerfacecolor('darkred')
        else:
            # Restaurar colores normales
            robot_markers[0].set_markerfacecolor(colors[0])
            robot_markers[1].set_markerfacecolor(colors[1])
    
    # Actualizar textos informativos
    time_text.set_text(f'Tiempo: {frame}')
    target_text.set_text(f'Objetivos alcanzados - Robot 1: {targets_reached[0]}, Robot 2: {targets_reached[1]}')
    
    return robot_markers + robot_paths + [time_text, target_text] + target_circles

# Configuración del gráfico de simulación
ax_sim.set_xlim(-2.5, 2.5)
ax_sim.set_ylim(-2.5, 2.5)
ax_sim.set_aspect('equal')
ax_sim.grid(True)
ax_sim.set_title('Simulación del movimiento de los robots')

# Crear la animación
from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig_sim, update_animation, frames=max_length,
                    init_func=init_animation, blit=True, interval=100)

# Guardar la animación como un video
print("Guardando animación como video...")
ani.save('simulacion_robots.mp4', writer='ffmpeg', fps=10, dpi=200)

# Mostrar la animación
plt.show() 