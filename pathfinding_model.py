from mesa import Agent, Model
from mesa.space import MultiGrid
from mesa.time import BaseScheduler
from mesa.datacollection import DataCollector

class Package:
    """Representa un paquete que debe ser recogido y entregado"""
    def __init__(self, package_id, pickup_location, delivery_location):
        self.id = package_id
        self.pickup_location = pickup_location  # Posición de recogida (camión)
        self.delivery_location = delivery_location  # Posición de entrega (góndola)
        self.status = 'waiting'  # waiting, assigned, picked, delivered
        self.assigned_robot_id = None
        self.pickup_time = None
        self.delivery_time = None

class ObstacleAgent(Agent):
    """Agente que representa un obstáculo en el grid"""
    def __init__(self, unique_id, model):        
        super().__init__(unique_id, model)
    
    def step(self):
        # Los obstáculos no hacen nada durante los pasos
        pass

class ChargingStation:
    """Representa una estación de carga (no es un agente)"""
    def __init__(self, position, charging_rate=10):
        self.pos = position  # Posición en el grid
        self.charging_rate = charging_rate  # Tasa de carga por paso

class RobotAgent(Agent):
    def __init__(self, unique_id, model, start, goal, color="red", 
                 max_battery=100, battery_drain_rate=1, battery_level=None):
        super().__init__(unique_id, model)
        
        # Convertir listas a tuplas si es necesario
        if isinstance(start, list):
            start = tuple(start)
        if isinstance(goal, list):
            goal = tuple(goal)
            
        self.start = start
        self.goal = goal
        self.path = self.astar(start, goal)
        self.steps_taken = 0
        self.color = color  # Añadimos un color para identificar cada robot
        self.reached_goal = False
        
        # Parámetros de batería
        self.max_battery = max_battery
        self.battery_level = battery_level if battery_level is not None else max_battery
        self.battery_drain_rate = battery_drain_rate
        self.charging = False
        self.low_battery_threshold = 30  # % de batería para buscar estación de carga
        self.nearest_charging_station = None
        self.original_path = self.path.copy() if self.path else []
        self.blocked_count = 0  # Contador para cuando el robot está bloqueado
        self.waiting_time = 0   # Tiempo de espera cuando hay bloqueo
        self.last_position = None  # Para detectar si el robot está atascado
        self.position_unchanged_count = 0  # Contador de pasos en los que no ha cambiado de posición
        self.alternative_paths_tried = []  # Guardar rutas alternativas ya intentadas
        self.priority = 1  # Prioridad base del robot (mayor número = mayor prioridad)
        self.returning_to_task = False
        # Añadir estas líneas al final del constructor:
        self.carrying_package = None  # Paquete que lleva el robot
        self.package_destination = None  # Destino del paquete (recogida o entrega)
        self.total_packages_delivered = 0  # Contador de paquetes entregados por este robot
        self.idle = True  # Por defecto, el robot empieza en estado idle
        
        if not self.path:
            print(f"Robot {unique_id}: No se encontró camino del inicio al objetivo.")
        else:
            print(f"Robot {unique_id}: Ruta calculada: {self.path}")
            print(f"Robot {unique_id}: Nivel de batería: {self.battery_level}%")
    
    def astar(self, start, goal):
        def heuristic(a, b):
            # Distancia Manhattan adaptada al nuevo sistema de coordenadas
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        open_set = [start]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            open_set.remove(current)
            # Definimos los movimientos posibles: derecha, izquierda, arriba, abajo
            # En este sistema, x aumenta hacia la izquierda
            for d in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
                neighbor = (current[0] + d[0], current[1] + d[1])
                
                # Verificar si está dentro de los límites del grid
                if 0 <= neighbor[0] < self.model.grid.width and 0 <= neighbor[1] < self.model.grid.height:
                    # Verificar si hay obstáculos
                    if not self.model.has_obstacle(neighbor):
                        # Verificar si hay otros robots en la posición
                        # Solo considerar como bloqueado si el robot no está en movimiento
                        robot_blocking = False
                        for robot in self.model.robots:
                            if robot.unique_id != self.unique_id and robot.pos == neighbor and neighbor != robot.goal:
                                robot_blocking = True
                                break
                        
                        if not robot_blocking:
                            tentative_g_score = g_score[current] + 1
                            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                                came_from[neighbor] = current
                                g_score[neighbor] = tentative_g_score
                                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                                if neighbor not in open_set:
                                    open_set.append(neighbor)
        return []
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.insert(0, current)
        return path

    def change_goal(self, new_goal):
        """
        Cambia la meta del robot y recalcula la ruta
        
        Args:
            new_goal: La nueva posición objetivo (tupla o lista)
            
        Returns:
            bool: True si se encontró una ruta, False en caso contrario
        """
        # Convertir lista a tupla si es necesario
        if isinstance(new_goal, list):
            new_goal = tuple(new_goal)
            
        # Establecer la nueva meta
        self.goal = new_goal
        
        # Resetear los indicadores relacionados con la meta
        self.reached_goal = False
        self.returning_to_task = False
        
        # Calcular la nueva ruta desde la posición actual hasta la nueva meta
        self.path = self.astar(self.pos, new_goal)
        
        # Retornar si se encontró una ruta
        return len(self.path) > 0
    def assign_package(self, package):
        """Asigna un paquete al robot"""
        self.carrying_package = package
        package.assigned_robot_id = self.unique_id
        package.status = 'assigned'
        # Establecer el punto de recogida como destino
        self.package_destination = package.pickup_location
        # Cambiar la meta a la ubicación de recogida
        self.change_goal(package.pickup_location)
        # Aumentar la prioridad cuando tiene un paquete asignado
        self.priority = 2
        # Cambiar el estado idle a False si existe
        if hasattr(self, 'idle'):
            self.idle = False
        print(f"Robot {self.unique_id}: Asignado paquete {package.id}. Dirigiéndose a recogerlo.")
    
    def pick_package(self):
        """El robot recoge un paquete del punto de recogida"""
        if self.carrying_package and self.carrying_package.status == 'assigned':
            self.carrying_package.status = 'picked'
            self.carrying_package.pickup_time = self.model.schedule.steps
            # Actualizar destino al punto de entrega
            self.package_destination = self.carrying_package.delivery_location
            # Cambiar la meta al punto de entrega
            self.change_goal(self.carrying_package.delivery_location)
            self.priority = 3
            print(f"Robot {self.unique_id}: Recogió paquete {self.carrying_package.id}. Dirigiéndose a entregarlo.")
            return True
        return False
    
    def deliver_package(self):
        """El robot entrega un paquete en el punto de entrega"""
        if self.carrying_package and self.carrying_package.status == 'picked':
            self.carrying_package.status = 'delivered'
            self.carrying_package.delivery_time = self.model.schedule.steps
            # Añadir a las estadísticas del modelo
            self.model.delivered_packages.append(self.carrying_package)
            # Incrementar contador del robot
            self.total_packages_delivered += 1
            # Limpiar estado
            delivered_package = self.carrying_package
            self.carrying_package = None
            self.package_destination = None
            # Volver a prioridad normal
            self.priority = 1
            # Volver a estado idle si corresponde
            if hasattr(self, 'idle'):
                self.idle = True
                self.path = []
            print(f"Robot {self.unique_id}: Entregó paquete {delivered_package.id}. Listo para nueva tarea.")
            return True
        return False
    
    def check_package_status(self):
        """
        Verifica si el robot está en un punto de recogida o entrega
        y actúa en consecuencia
        """
        if self.carrying_package:
            # Si está en el punto de recogida y el paquete está asignado
            if (self.pos == self.package_destination and 
                self.carrying_package.status == 'assigned'):
                self.pick_package()
                return True
                
            # Si está en el punto de entrega y el paquete está recogido
            elif (self.pos == self.package_destination and 
                  self.carrying_package.status == 'picked'):
                self.deliver_package()
                return True
        return False
    
    def get_battery_percentage(self):
        """Devuelve el porcentaje de batería actual"""
        return (self.battery_level / self.max_battery) * 100
        
    def find_nearest_charging_station(self):
        """Encuentra la estación de carga más cercana"""
        if not self.model.charging_stations:
            return None
            
        # Encuentra la estación más cercana
        nearest = None
        min_distance = float('inf')
        
        for station in self.model.charging_stations:
            # Distancia Manhattan adaptada al nuevo sistema
            distance = abs(self.pos[0] - station.pos[0]) + abs(self.pos[1] - station.pos[1])
            if distance < min_distance:
                min_distance = distance
                nearest = station
        
        return nearest
    
    def calculate_path_to_station(self, station):
        """Calcula una ruta hacia la estación de carga"""
        if station is None:
            return []
            
        return self.astar(self.pos, station.pos)
    
    def charge_battery(self, amount):
        """Carga la batería con la cantidad especificada"""
        self.battery_level = min(self.max_battery, self.battery_level + amount)
        print(f"Robot {self.unique_id}: Cargando batería. Nivel actual: {self.battery_level:.1f}%")
    
    def drain_battery(self, amount=None):
        """Consume batería con la cantidad especificada o la tasa predeterminada"""
        if amount is None:
            amount = self.battery_drain_rate
            
        self.battery_level = max(0, self.battery_level - amount)
        
        # Si la batería se agota completamente, el robot se detiene
        if self.battery_level <= 0:
            print(f"Robot {self.unique_id}: ¡BATERÍA AGOTADA! Robot detenido.")
            return False
            
        # Si la batería está baja pero aún no se está cargando, buscar estación
        battery_percentage = self.get_battery_percentage()
        if battery_percentage <= self.low_battery_threshold and not self.charging and not self.nearest_charging_station:
            print(f"Robot {self.unique_id}: Batería baja ({battery_percentage:.1f}%). Buscando estación de carga...")
            self.nearest_charging_station = self.find_nearest_charging_station()
            
            if self.nearest_charging_station:
                # Guardar el camino original si aún no se ha llegado a la meta
                if not self.reached_goal:
                    self.original_path = self.path.copy()
                    
                # Calcular camino a la estación
                self.path = self.calculate_path_to_station(self.nearest_charging_station)
                if self.path:
                    print(f"Robot {self.unique_id}: Redirigiendo a estación de carga. Nueva ruta calculada.")
                else:
                    print(f"Robot {self.unique_id}: No se pudo encontrar ruta a la estación de carga.")
                    self.nearest_charging_station = None
            else:
                print(f"Robot {self.unique_id}: No hay estaciones de carga disponibles.")
                
        return True  # Batería suficiente para seguir funcionando
    
    def is_at_charging_station(self):
        """Verifica si el robot está en una estación de carga"""
        for station in self.model.charging_stations:
            if self.pos == station.pos:
                return station
        return None
    
    def step(self):
        if hasattr(self, 'idle') and self.idle:
            return

        if self.carrying_package and self.pos == self.package_destination:
            if self.check_package_status():
                return  # Si se recogió/entregó un paquete, terminar el paso

        # Si ya llegó a la meta, no hacer nada
        if self.reached_goal and not self.charging:
            return
            
        # Si está en una estación de carga
        if self.charging:
            # Verificar si sigue en la estación
            station = self.is_at_charging_station()
            
            if station:
                # Cargar batería
                self.charge_battery(station.charging_rate)
                
                # Si la batería está completa, continuar con la tarea original
                if self.battery_level >= self.max_battery * 0.95:  # 95% de carga
                    self.charging = False
                    self.nearest_charging_station = None
                    
                    # Si aún no ha llegado a la meta, retomar el camino original
                    if not self.reached_goal and self.original_path:
                        print(f"Robot {self.unique_id}: Batería cargada. Retomando tarea original.")
                        # Calcular nuevo camino desde la posición actual hasta la meta original
                        self.path = self.astar(self.pos, self.goal)
                        self.returning_to_task = True
            else:
                # Si no está en una estación de carga pero estaba en modo carga, algo salió mal
                self.charging = False
                print(f"Robot {self.unique_id}: Error: No se encontró estación de carga en la posición actual.")
            

            # Actualizar contador de posición sin cambios
        if self.last_position == self.pos:
            self.position_unchanged_count += 1
        else:
            self.position_unchanged_count = 0
            self.last_position = self.pos
        
        # Si lleva demasiado tiempo sin moverse, aumentar prioridad y buscar rutas alternativas
        if self.position_unchanged_count > 5:
            self.priority += 1  # Aumentar prioridad cada vez que está bloqueado
            print(f"Robot {self.unique_id}: Posiblemente bloqueado, aumentando prioridad a {self.priority}")
            
            # Buscar una ruta completamente nueva si está bloqueado demasiado tiempo
            if self.position_unchanged_count > 10:
                self.find_alternative_route()

        # Movimiento normal (si no está cargando)
        elif len(self.path) > 1:
            # Verificar si hay suficiente batería para moverse
            if not self.drain_battery():
                return  # Batería agotada, no moverse
                
            next_pos = self.path[1]  # El siguiente paso en la ruta
            
                # Verificar si el siguiente paso está ocupado por otro robot
            blocking_robot = None
            for robot in self.model.robots:
                if robot.unique_id != self.unique_id and robot.pos == next_pos:
                    blocking_robot = robot
                    break
            
            if blocking_robot is None:
                # El camino está libre, moverse normalmente
                self.blocked_count = 0  # Resetear contador de bloqueo
                self.waiting_time = 0   # Resetear tiempo de espera
                
                self.path.pop(0)
                next_pos = self.path[0]
                self.model.grid.move_agent(self, next_pos)
                self.steps_taken += 1
                
                # Verificar si llegó a una estación de carga
                station = self.is_at_charging_station()
                
                if station and self.nearest_charging_station:
                    print(f"Robot {self.unique_id}: Llegó a estación de carga.")
                    self.charging = True
                elif self.pos == self.goal and not self.charging and not self.returning_to_task:
                    self.reached_goal = True
                    print(f"¡Robot {self.unique_id} ha alcanzado el objetivo! (Batería: {self.battery_level:.1f}%)")
                else:
                    status = "Cargando" if self.charging else "Retornando a tarea" if self.returning_to_task else "Normal"
                    print(f"Robot {self.unique_id} se movió a {next_pos} (Paso {self.steps_taken}, Batería: {self.battery_level:.1f}%, Estado: {status})")
            else:
                # Camino bloqueado por otro robot
                self.blocked_count += 1
                
                # Determinar qué robot tiene mayor prioridad
                if self.priority > blocking_robot.priority or (self.priority == blocking_robot.priority and self.unique_id < blocking_robot.unique_id):
                    # Este robot tiene mayor prioridad
                    print(f"Robot {self.unique_id} (prioridad {self.priority}) está bloqueado por Robot {blocking_robot.unique_id} (prioridad {blocking_robot.priority})")
                    
                    # Si está bloqueado por poco tiempo, esperar
                    if self.blocked_count < 3:
                        print(f"Robot {self.unique_id} esperando {self.blocked_count} turno(s)...")
                        return
                    
                    # Después de esperar, buscar ruta alternativa
                    self.find_alternative_route()
                else:
                    # El otro robot tiene mayor prioridad, esperar un poco más
                    self.waiting_time += 1
                    if self.waiting_time > 2:
                        # Después de esperar, buscar ruta alternativa
                        print(f"Robot {self.unique_id} cede el paso a Robot {blocking_robot.unique_id} y busca ruta alternativa")
                        self.find_alternative_route()
        elif len(self.path) == 1 and self.pos == self.path[0]:
            # Si llegó al final de la ruta
            if self.pos == self.goal and not self.returning_to_task:
                self.reached_goal = True
                print(f"¡Robot {self.unique_id} ha alcanzado el objetivo! (Batería: {self.battery_level:.1f}%)")
            
            # Verificar si está en una estación de carga
            station = self.is_at_charging_station()
            
            if station and not self.charging and self.nearest_charging_station:
                print(f"Robot {self.unique_id}: Llegó a estación de carga.")
                self.charging = True
    
    def find_alternative_route(self):
        """Busca una ruta alternativa cuando el robot está bloqueado"""
        print(f"Robot {self.unique_id}: Buscando ruta alternativa...")
        
        # Guardar la ruta actual para compararla
        old_path = self.path.copy() if self.path else []
        
        # Determinar destino final (meta del robot o estación de carga)
        destination = self.goal
        if self.charging and self.nearest_charging_station:
            destination = self.nearest_charging_station.pos
        
        # Estrategia 1: Ruta normal A* (ya intentada pero volvemos a intentar por si hay cambios)
        self.path = self.astar(self.pos, destination)
        
        # Si no encontró ruta o es la misma, probar con penalización de posiciones ocupadas
        if not self.path or self.path == old_path or self.path_in_tried_alternatives(self.path):
            print(f"Robot {self.unique_id}: Buscando ruta con penalización de robots...")
            self.path = self.astar_with_robot_penalty(self.pos, destination)
        
        # Si aún no encuentra ruta, intentar una ruta más larga con desvío
        if not self.path or self.path == old_path or self.path_in_tried_alternatives(self.path):
            print(f"Robot {self.unique_id}: Buscando ruta con desvío...")
            self.path = self.find_path_with_detour(self.pos, destination)
        
        # Si sigue sin encontrar ruta, esperar y mantener la ruta anterior
        if not self.path:
            print(f"Robot {self.unique_id}: No se encontró ruta alternativa, manteniendo la actual y esperando...")
            self.path = old_path
            return False
        
        # Guardar esta ruta en las alternativas probadas
        if len(self.alternative_paths_tried) > 3:
            self.alternative_paths_tried.pop(0)  # Eliminar la más antigua
        self.alternative_paths_tried.append([tuple(pos) for pos in self.path])
        
        # Resetear contadores
        self.blocked_count = 0
        self.waiting_time = 0
        
        # Imprimir información sobre la nueva ruta
        if self.path != old_path:
            print(f"Robot {self.unique_id}: Ruta recalculada con éxito, longitud: {len(self.path)}")
            return True
        else:
            print(f"Robot {self.unique_id}: No se encontró una ruta mejor")
            return False

    def path_in_tried_alternatives(self, path):
        """Verifica si una ruta ya fue probada anteriormente"""
        if not path:
            return False
        
        path_tuples = [tuple(pos) for pos in path]
        
        for tried_path in self.alternative_paths_tried:
            if len(tried_path) == len(path_tuples):
                matches = True
                for i in range(len(tried_path)):
                    if tried_path[i] != path_tuples[i]:
                        matches = False
                        break
                if matches:
                    return True
        
        return False

    def astar_with_robot_penalty(self, start, goal):
        """A* con penalización adicional por celdas cercanas a robots"""
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        # Crear un mapa de penalizaciones basado en posiciones de robots
        robot_penalty_map = {}
        for robot in self.model.robots:
            if robot.unique_id != self.unique_id:
                # Penalizar la posición del robot
                robot_penalty_map[robot.pos] = 10
                
                # Penalizar posiciones adyacentes
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    pos = (robot.pos[0] + dx, robot.pos[1] + dy)
                    if 0 <= pos[0] < self.model.grid.width and 0 <= pos[1] < self.model.grid.height:
                        robot_penalty_map[pos] = robot_penalty_map.get(pos, 0) + 5
        
        open_set = [start]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            open_set.remove(current)
            
            for d in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
                neighbor = (current[0] + d[0], current[1] + d[1])
                
                if 0 <= neighbor[0] < self.model.grid.width and 0 <= neighbor[1] < self.model.grid.height:
                    if not self.model.has_obstacle(neighbor):
                        # Base cost
                        move_cost = 1
                        
                        # Additional penalty if position is near robots
                        move_cost += robot_penalty_map.get(neighbor, 0)
                        
                        tentative_g_score = g_score[current] + move_cost
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                            if neighbor not in open_set:
                                open_set.append(neighbor)
        return []
    
    def find_path_with_detour(self, start, goal):
        """Busca un camino con desvío para evitar bloqueos"""
        # Encontrar un punto intermedio para desvío
        detour_points = []
        
        # Calcular la dirección general hacia la meta
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        
        # Generar puntos de desvío perpendiculares a la dirección principal
        if abs(dx) > abs(dy):  # Movimiento principalmente horizontal
            # Intentar desvíos verticales
            detour_points.append((start[0], start[1] + 3))
            detour_points.append((start[0], start[1] - 3))
        else:  # Movimiento principalmente vertical
            # Intentar desvíos horizontales
            detour_points.append((start[0] + 3, start[1]))
            detour_points.append((start[0] - 3, start[1]))
        
        # Añadir algunos puntos aleatorios
        import random
        for _ in range(2):
            random_dx = random.randint(-5, 5)
            random_dy = random.randint(-5, 5)
            detour_points.append((start[0] + random_dx, start[1] + random_dy))
        
        # Filtrar puntos fuera de los límites o con obstáculos
        valid_detour_points = []
        for point in detour_points:
            if (0 <= point[0] < self.model.grid.width and 
                0 <= point[1] < self.model.grid.height and
                not self.model.has_obstacle(point)):
                valid_detour_points.append(point)
        
        # Intentar encontrar un camino con desvío
        for detour_point in valid_detour_points:
            # Camino hasta el punto de desvío
            path_to_detour = self.astar(start, detour_point)
            if not path_to_detour:
                continue
                
            # Camino desde el desvío hasta la meta
            path_from_detour = self.astar(detour_point, goal)
            if not path_from_detour:
                continue
                
            # Combinar los caminos (eliminar duplicado del punto de desvío)
            combined_path = path_to_detour + path_from_detour[1:]
            print(f"Robot {self.unique_id}: Ruta con desvío encontrada a través de {detour_point}")
            return combined_path
        
        # Si no se encontró ningún camino con desvío
        return []

    

class PathFindingModel(Model):
    def __init__(self, width, height, robot_configs, charging_station_positions=None):
        super().__init__()
        self.grid = MultiGrid(width, height, torus=False)
        self.schedule = BaseScheduler(self)
        self.obstacles = []  # Lista para almacenar los agentes obstáculo
        self.robots = []  # Lista para almacenar los robots
        self.charging_stations = []  # Lista para almacenar las estaciones de carga
        self.packages = []  # Lista de todos los paquetes
        self.delivered_packages = []  # Lista de paquetes entregados
        self.next_package_id = 1  # ID para el siguiente paquete
        
        # Crear y registrar las estaciones de carga (no son agentes)
        if charging_station_positions:
            for pos in charging_station_positions:
                if isinstance(pos, list):
                    pos = tuple(pos)
                    
                station = ChargingStation(pos)
                self.charging_stations.append(station)
        
        # Crear y colocar los robots
        robot_id = 1
        for config in robot_configs:
            # Extraer configuración del robot
            start = config['start']
            goal = config['goal']
            color = config.get('color', "red")  # Color por defecto
            
            # Extraer parámetros de batería si existen
            max_battery = config.get('max_battery', 100)
            battery_drain_rate = config.get('battery_drain_rate', 1)
            battery_level = config.get('battery_level', max_battery)
            
            robot = RobotAgent(
                robot_id, self, start, goal, color,
                max_battery=max_battery,
                battery_drain_rate=battery_drain_rate,
                battery_level=battery_level
            )
            self.robots.append(robot)
            self.schedule.add(robot)
            self.grid.place_agent(robot, tuple(start) if isinstance(start, list) else start)
            robot_id += 1
        
        # Añadir recolector de datos para estadísticas
        self.datacollector = DataCollector(
            model_reporters={
                "Pasos Promedio": lambda m: sum(robot.steps_taken for robot in m.robots) / len(m.robots) if m.robots else 0,
                "Robots en Meta": lambda m: sum(1 for robot in m.robots if robot.reached_goal),
                "Batería Promedio": lambda m: sum(robot.battery_level for robot in m.robots) / len(m.robots) if m.robots else 0
            },
            agent_reporters={
                "Batería": lambda a: a.battery_level if hasattr(a, "battery_level") else None,
                "Posición": lambda a: a.pos,
                "Pasos": lambda a: a.steps_taken if hasattr(a, "steps_taken") else None
            }
        )
    
    def create_package(self, pickup_location, delivery_location):
        """Crea un nuevo paquete"""
        package = Package(self.next_package_id, pickup_location, delivery_location)
        self.next_package_id += 1
        self.packages.append(package)
        return package

    def get_available_packages(self):
        """Retorna los paquetes disponibles para asignación"""
        return [p for p in self.packages if p.status == 'waiting']
    
    def get_truck_positions(self):
        """Retorna una lista de todas las posiciones de los camiones (a implementar en server.py)"""
        # Este método se implementará en el servidor
        pass
    
    def get_delivery_positions(self):
        """Retorna una lista de todos los puntos de entrega (a implementar en server.py)"""
        # Este método se implementará en el servidor
        pass
    
    def assign_package_to_robot(self, package_id, robot_id):
        """Asigna un paquete a un robot específico"""
        package = next((p for p in self.packages if p.id == package_id), None)
        robot = next((r for r in self.robots if r.unique_id == robot_id), None)
        
        if not package or not robot:
            return False
            
        if package.status != 'waiting' or robot.carrying_package:
            return False
        
        robot.assign_package(package)
        # Cambiar el estado idle a False
        robot.idle = False
        return True
        
    def has_obstacle(self, pos):
        """Comprueba si hay un obstáculo en la posición dada"""
        # Asegurar que pos sea una tupla
        if isinstance(pos, list):
            pos = tuple(pos)
            
        cell_contents = self.grid.get_cell_list_contents(pos)
        for agent in cell_contents:
            if isinstance(agent, ObstacleAgent):
                return True
        return False
    
    def add_obstacle(self, pos):
        """Añade un obstáculo en la posición especificada"""
        # Asegurar que pos sea una tupla
        if isinstance(pos, list):
            pos = tuple(pos)
            
        # Verificar que la posición no sea ni la de inicio ni la de meta de ningún robot
        for robot in self.robots:
            if pos == robot.start or pos == robot.goal:
                return False
        
        # Verificar que no sea una estación de carga
        for station in self.charging_stations:
            if pos == station.pos:
                return False
                
        if not self.has_obstacle(pos):
            # Crear un nuevo agente obstáculo
            obstacle_id = len(self.obstacles) + 100  # IDs únicos para obstáculos
            obstacle = ObstacleAgent(obstacle_id, self)
            self.obstacles.append(obstacle)
            self.schedule.add(obstacle)
            self.grid.place_agent(obstacle, pos)
            
            # Recalcular la ruta de todos los robots
            for robot in self.robots:
                if not robot.reached_goal and robot.path:
                    if robot.charging and robot.nearest_charging_station:
                        # Si se está dirigiendo a una estación de carga, recalcular esa ruta
                        robot.path = robot.calculate_path_to_station(robot.nearest_charging_station)
                    else:
                        # Si no, recalcular la ruta normal
                        robot.path = robot.astar(robot.pos, robot.goal)
            
            return True
        
        return False
    
    def add_charging_station(self, pos):
        """Añade una estación de carga en la posición especificada"""
        # Asegurar que pos sea una tupla
        if isinstance(pos, list):
            pos = tuple(pos)
            
        # Verificar que no hay obstáculos en la posición
        if self.has_obstacle(pos):
            return False
            
        # Verificar que no hay otra estación de carga en la misma posición
        for station in self.charging_stations:
            if station.pos == pos:
                return False
                
        # Crear nueva estación de carga (no es un agente)
        station = ChargingStation(pos)
        self.charging_stations.append(station)
        
        return True
    
    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()
    
    def all_robots_reached_goal(self):
        """Verifica si todos los robots han alcanzado su meta"""
        return all(robot.reached_goal for robot in self.robots)