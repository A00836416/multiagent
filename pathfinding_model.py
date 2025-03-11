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
        self.waiting_queue = []  # Cola de robots esperando para cargar
        self.current_robot = None  # Robot actualmente cargando

    def add_to_queue(self, robot_id):
        """Añade un robot a la cola de espera si no está ya"""
        if robot_id not in self.waiting_queue and robot_id != self.current_robot:
            self.waiting_queue.append(robot_id)
            print(f"Robot {robot_id} añadido a la cola de la estación en {self.pos}. Cola actual: {self.waiting_queue}")
            return True
        return False

    def is_next_in_queue(self, robot_id):
        """Verifica si el robot es el siguiente en la cola"""
        if self.current_robot is None and len(self.waiting_queue) > 0:
            return self.waiting_queue[0] == robot_id
        return False

    def start_charging(self, robot_id):
        """Comienza a cargar un robot y lo elimina de la cola"""
        if self.current_robot is None:
            if robot_id in self.waiting_queue:
                self.waiting_queue.remove(robot_id)
            self.current_robot = robot_id
            print(f"Robot {robot_id} comienza a cargar en la estación {self.pos}")
            return True
        return False

    def finish_charging(self, robot_id):
        """Finaliza la carga de un robot"""
        if self.current_robot == robot_id:
            self.current_robot = None
            print(f"Robot {robot_id} terminó de cargar en la estación {self.pos}")
            return True
        return False

    def remove_from_queue(self, robot_id):
        """Elimina un robot de la cola (si cancela su carga)"""
        if robot_id in self.waiting_queue:
            self.waiting_queue.remove(robot_id)
            print(f"Robot {robot_id} eliminado de la cola de la estación {self.pos}")
            return True
        return False

class RobotAgent(Agent):
    def __init__(self, unique_id, model, start, goal, color="red", 
                 max_battery=100, battery_drain_rate=0.5, battery_level=None):
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
        self.low_battery_threshold = 35  # % de batería para buscar estación de carga
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

        # Añadir variables para prevenir bucles de carga
        self.just_charged = False  # Indica si acaba de salir de una estación de carga
        self.charge_cooldown = 0   # Contador de pasos desde la última carga
        # Añadir modo de ahorro de energía
        self.energy_saving_mode = False
        self.energy_saving_drain_rate = 0.3  # Tasa reducida en modo ahorro
        self.waiting_for_charge = False  # Indica si está esperando para cargar
        self.charging_station_target = None  # Guarda la referencia a la estación objetivo
        self.critical_battery = False  # Indica si la batería está en nivel crítico
        self.emergency_route = False   # Indica si está en ruta de emergencia a una estación
        self.critical_battery_threshold = 20  # Medidas especiales al 20%
        self.emergency_battery_threshold = 10  # Acciones drásticas al 10%
        
        if not self.path:
            print(f"Robot {unique_id}: No se encontró camino del inicio al objetivo.")
        else:
            print(f"Robot {unique_id}: Ruta calculada: {self.path}")
            print(f"Robot {unique_id}: Nivel de batería: {self.battery_level}%")
    
    
    def calculate_emergency_path(self, start, goal):
        """Calcula una ruta de emergencia directa hacia la estación de carga.
        Usa A* simple sin preocuparse tanto por penalizaciones de robots."""
        
        def heuristic(a, b):
            # Distancia Manhattan
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        open_set = [start]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == goal:
                # Reconstruir el camino
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.insert(0, current)
                print(f"Robot {self.unique_id}: Ruta de emergencia encontrada, longitud: {len(path)}")
                return path
            
            open_set.remove(current)
            
            # Explorar los 4 movimientos cardinales básicos
            for d in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
                neighbor = (current[0] + d[0], current[1] + d[1])
                
                # Verificar límites del grid
                if 0 <= neighbor[0] < self.model.grid.width and 0 <= neighbor[1] < self.model.grid.height:
                    # Solo evitar obstáculos, ignorar otros robots
                    if not self.model.has_obstacle(neighbor):
                        tentative_g_score = g_score[current] + 1
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                            if neighbor not in open_set:
                                open_set.append(neighbor)
        
        # Si no se encuentra ruta, intentar con el método normal
        print(f"Robot {self.unique_id}: No se pudo encontrar ruta de emergencia, usando método normal")
        return self.calculate_path_to_station(self.find_nearest_charging_station())
    
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
    
    def check_robots_health(self):
        """Verifica periódicamente el estado de todos los robots"""
        for robot in self.robots:
            # Detectar robots con batería crítica no dirigiéndose a cargar
            if robot.battery_level < robot.max_battery * 0.15 and not robot.charging and not robot.nearest_charging_station:
                print(f"Robot {robot.unique_id}: ALERTA DE SISTEMA - Batería crítica.")
                # Forzar búsqueda de estación
                nearest_station = robot.find_emergency_charging_station()
                if nearest_station:
                    robot.nearest_charging_station = nearest_station
                    robot.charging_station_target = nearest_station
                    robot.path = robot.calculate_emergency_path(robot.pos, nearest_station.pos)
            
            # Detectar robots atascados
            if hasattr(robot, 'position_unchanged_count') and robot.position_unchanged_count > 10:
                print(f"Robot {robot.unique_id}: ATENCIÓN - Robot atascado por {robot.position_unchanged_count} pasos.")
                # Forzar reseteo de estado
                robot.find_alternative_route()
                robot.priority += 5  # Aumentar prioridad dramáticamente

    def handle_charging_station_arrival(self):
        """Maneja la llegada a una estación de carga y la carga de batería"""
        # Buscar si estamos en una estación de carga
        station = next((s for s in self.model.charging_stations if s.pos == self.pos), None)
        
        if not station:
            # Si no estamos en una estación, limpiar los estados relacionados con la carga
            self.waiting_for_charge = False
            self.charging = False
            self.current_charging_station = None
            return False
        
        # Si ya estamos cargando, continuar con ese proceso
        if self.charging:
            if self.battery_level >= self.max_battery:
                # Terminar la carga si la batería está llena
                self.charging = False
                if self.current_charging_station:
                    self.current_charging_station.finish_charging(self.unique_id)
                    self.current_charging_station = None
                print(f"Robot {self.unique_id}: Carga completada, batería al 100%")
                return True
            else:
                # Continuar cargando
                self.charge_battery(station.charging_rate)
                return True
        
        # Verificar si podemos comenzar a cargar
        if station.is_next_in_queue(self.unique_id):
            success = station.start_charging(self.unique_id)
            if success:
                self.charging = True
                self.waiting_for_charge = False
                self.current_charging_station = station
                print(f"Robot {self.unique_id}: Comenzando a cargar en {station.pos}")
                return True
        
        # Si estamos esperando en la cola
        if self.waiting_for_charge:
            # En lugar de intentar acceder directamente a station.queue, 
            # usamos un método existente para verificar si seguimos en la cola
            if station.is_next_in_queue(self.unique_id) or self.is_in_station_queue(station):
                print(f"Robot {self.unique_id}: Esperando en cola de estación {station.pos}")
                
                # Si tenemos batería crítica, aumentar prioridad periódicamente
                if self.battery_level < self.max_battery * 0.1:
                    self.priority += 1
                    print(f"Robot {self.unique_id}: Batería crítica en espera. Aumentando prioridad a {self.priority}")
                return True
            else:
                # Ya no estamos en la cola, restablecer el estado de espera
                self.waiting_for_charge = False
        
       
            
        return False

    def is_in_station_queue(self, station):
        """Comprueba si el robot está en la cola de la estación sin acceder directamente a la cola"""
        # Implementamos un método auxiliar que verifica si el robot está en la cola
        # sin acceder directamente al atributo queue
        try:
            # Intentar añadirnos a la cola y ver si falla
            # Si ya estamos en la cola, debería fallar o retornar False
            result = station.add_to_queue(self.unique_id)
            if result is False:  # Si el método retorna False explícitamente
                return True      # Estamos ya en la cola
            else:
                # Si tuvo éxito añadiéndonos, nos quitamos inmediatamente
                station.remove_from_queue(self.unique_id)
                return False
        except Exception:
            # Si hubo un error (como intentar añadir un robot que ya está),
            # asumimos que estamos en la cola
            return True

    def prioritize_charging_stations(self):
        """Busca y prioriza estaciones de carga basado en la ocupación y distancia"""
        
        if not self.model.charging_stations:
            return None
        
        # Encontrar todas las estaciones ordenadas por distancia
        stations_by_distance = []
        for station in self.model.charging_stations:
            # Calcular distancia Manhattan
            distance = abs(self.pos[0] - station.pos[0]) + abs(self.pos[1] - station.pos[1])
            
            # Considerar si hay batería suficiente para llegar a esta estación
            # Añadir 10% más como margen de seguridad
            battery_needed = distance * self.battery_drain_rate * 1.1
            can_reach = self.battery_level >= battery_needed
            
            stations_by_distance.append((station, distance, can_reach))
        
        # Ordenar primero por accesibilidad (si puede llegar) y luego por distancia
        stations_by_distance.sort(key=lambda x: (0 if x[2] else 1, x[1]))
        
        # Si hay batería crítica (menos del 8%), ir a la estación más cercana a la que pueda llegar
        if self.battery_level < self.max_battery * 0.08:
            self.critical_battery = True
            self.emergency_route = True
            
            # Buscar la estación más cercana a la que pueda llegar
            for station, distance, can_reach in stations_by_distance:
                if can_reach:
                    print(f"Robot {self.unique_id}: ¡BATERÍA CRÍTICA! Dirigiéndose a estación en {station.pos} (batería necesaria: {distance * self.battery_drain_rate:.1f})")
                    return station
                    
            # Si no puede llegar a ninguna, intentar con la más cercana de todas formas
            if stations_by_distance:
                nearest_station = stations_by_distance[0][0]
                print(f"Robot {self.unique_id}: ¡ALERTA! Batería insuficiente pero intentando llegar a estación más cercana")
                return nearest_station
            return None
        
        # Para casos no críticos, considerar espera + distancia
        reachable_stations = [s for s in stations_by_distance if s[2]]
        if not reachable_stations:
            print(f"Robot {self.unique_id}: No hay estaciones accesibles con la batería actual")
            return None
            
        best_station = None
        min_wait_time = float('inf')
        
        for station, distance, _ in reachable_stations:
            # Estimar tiempo de espera: robots en cola + robot actual (si existe)
            wait_time = len(station.waiting_queue)
            if station.current_robot is not None:
                wait_time += 1
                
            # Añadir el tiempo de viaje (cada paso es una unidad de tiempo)
            total_wait = wait_time + distance
            
            if total_wait < min_wait_time:
                min_wait_time = total_wait
                best_station = station
        
        return best_station

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
        
        # Calcular la nueva ruta desde la posición actual hasta la nueva meta
        self.path = self.astar(self.pos, new_goal)
        
        # Si no se encontró una ruta, intentar métodos alternativos
        if not self.path or len(self.path) < 2:
            print(f"Robot {self.unique_id}: No se pudo encontrar ruta directa a {new_goal}. Intentando con penalización.")
            # Intentar con penalización de robots
            self.path = self.astar_with_robot_penalty(self.pos, new_goal)
            
            if not self.path or len(self.path) < 2:
                print(f"Robot {self.unique_id}: Fallido. Intentando con desvío.")
                # Intentar con desvío
                self.path = self.find_path_with_detour(self.pos, new_goal)
        
        # Garantizar que al menos tengamos la posición actual en la ruta
        if not self.path:
            self.path = [self.pos]
            return False
            
        # Retornar True si se encontró una ruta significativa (más de un punto)
        return len(self.path) > 1
    
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
            if self.energy_saving_mode:
                amount = self.energy_saving_drain_rate
            else:
                amount = self.battery_drain_rate
                    
        self.battery_level = max(0, self.battery_level - amount)
            
        # Si la batería se agota completamente, el robot se detiene
        if self.battery_level <= 0:
            print(f"Robot {self.unique_id}: ¡BATERÍA AGOTADA! Robot detenido.")
            # Reset del estado para evitar confusiones
            self.waiting_for_charge = False
            self.charging_station_target = None
            self.nearest_charging_station = None
            return False
            
        # Si la batería está muy baja (menos del 15%), activar modo ahorro de energía
        if self.battery_level < self.max_battery * 0.20 and not self.energy_saving_mode:
            self.energy_saving_mode = True
            print(f"Robot {self.unique_id}: ¡MODO AHORRO DE ENERGÍA ACTIVADO! Reduciendo consumo.")
                
        # Si la batería está extremadamente baja (menos del 8%) y no estamos en ruta a una estación
        if self.battery_level < self.max_battery * 0.08 and not self.nearest_charging_station and not self.critical_battery:
            self.critical_battery = True
            print(f"Robot {self.unique_id}: ¡NIVEL DE BATERÍA CRÍTICO! Buscando estación inmediatamente.")
            
            # Aumentar prioridad drásticamente para que otros robots cedan el paso
            self.priority = 10  # Prioridad muy alta para casos críticos
                
            # Buscar la estación más cercana con prioridad
            nearest_station = self.prioritize_charging_stations()
                
            if nearest_station:
                # Guardar el camino original si no se ha llegado a la meta
                if not self.reached_goal:
                    self.original_path = self.path.copy() if self.path else []
                        
                # Establecer la estación como destino de emergencia
                self.nearest_charging_station = nearest_station
                self.emergency_route = True
                self.path = self.calculate_path_to_station(nearest_station)
                
                # Importante: Intentar añadir a la cola solo si se pudo encontrar una ruta
                if self.path:
                    # Añadir a la cola de espera
                    nearest_station.add_to_queue(self.unique_id)
                    self.charging_station_target = nearest_station
                    print(f"Robot {self.unique_id}: RUTA DE EMERGENCIA a estación en {nearest_station.pos} ({len(self.path)} pasos)")
                else:
                    print(f"Robot {self.unique_id}: No se pudo encontrar ruta a la estación de carga.")
                    self.nearest_charging_station = None
                    self.critical_battery = False
            else:
                print(f"Robot {self.unique_id}: No hay estaciones de carga disponibles.")
            
        # Si la batería está baja pero aún no se está cargando, buscar estación
        battery_percentage = self.get_battery_percentage()
        if battery_percentage <= self.low_battery_threshold and not self.charging and not self.nearest_charging_station and not self.waiting_for_charge:
            print(f"Robot {self.unique_id}: Batería baja ({battery_percentage:.1f}%). Evaluando opciones...")
                
            # Buscar estación de carga considerando colas
            best_station = self.prioritize_charging_stations()
                
            if best_station:
                # Guardar el camino original si aún no se ha llegado a la meta
                if not self.reached_goal:
                    self.original_path = self.path.copy() if self.path else []
                        
                # Calcular ruta a la estación
                self.nearest_charging_station = best_station
                self.charging_station_target = best_station
                self.path = self.calculate_path_to_station(best_station)
                    
                if self.path:
                    # Añadir a la cola de espera (SOLO si hay ruta válida)
                    success = best_station.add_to_queue(self.unique_id)
                    if success:
                        # Verificar que el robot realmente esté en la cola antes de imprimir posición
                        if self.unique_id in best_station.waiting_queue:
                            queue_position = best_station.waiting_queue.index(self.unique_id) + 1
                            print(f"Robot {self.unique_id}: Redirigiendo a estación en {best_station.pos} ({len(self.path)} pasos). Posición en cola: {queue_position}")
                        else:
                            print(f"Robot {self.unique_id}: Redirigiendo a estación en {best_station.pos} ({len(self.path)} pasos).")
                    else:
                        print(f"Robot {self.unique_id}: Redirigiendo a estación en {best_station.pos} ({len(self.path)} pasos). No se pudo añadir a la cola.")
                else:
                    # Si no se puede encontrar ruta, limpiar referencias a la estación
                    self.nearest_charging_station = None
                    self.charging_station_target = None
                    print(f"Robot {self.unique_id}: No se pudo encontrar ruta a la estación de carga.")
            else:
                print(f"Robot {self.unique_id}: No hay estaciones de carga disponibles.")
                        
        return True  # Batería suficiente para seguir funcionando
    
    def is_at_charging_station(self):
        """Verifica si el robot está en una estación de carga"""
        for station in self.model.charging_stations:
            if self.pos == station.pos:
                return station
        return None

    def determine_priority_in_collision(self, other_robot):
        """
        Determina qué robot tiene mayor prioridad en una colisión.
        
        Args:
            other_robot: El otro robot involucrado en la colisión
            
        Returns:
            bool: True si este robot tiene mayor prioridad, False en caso contrario
        """
        # 1. Los robots en estado crítico de batería tienen la máxima prioridad
        if self.critical_battery and not other_robot.critical_battery:
            return True
        elif not self.critical_battery and other_robot.critical_battery:
            return False
        
        # 2. Los robots que van a una estación de carga con batería baja tienen prioridad
        if self.nearest_charging_station and self.battery_level < self.max_battery * 0.2 and not other_robot.nearest_charging_station:
            return True
        elif not self.nearest_charging_station and other_robot.nearest_charging_station and other_robot.battery_level < other_robot.max_battery * 0.2:
            return False
        
        # 3. Los robots con menor batería porcentual tienen prioridad entre robots que buscan estación
        if self.nearest_charging_station and other_robot.nearest_charging_station:
            self_percentage = self.battery_level / self.max_battery
            other_percentage = other_robot.battery_level / other_robot.max_battery
            if self_percentage < other_percentage:
                return True
            elif other_percentage < self_percentage:
                return False
        
        # 4. Los robots que llevan paquetes tienen prioridad sobre los que no
        if self.carrying_package and not other_robot.carrying_package:
            return True
        elif not self.carrying_package and other_robot.carrying_package:
            return False
        
        # 5. Si ambos llevan paquete, el que tenga prioridad numérica mayor gana
        if self.priority > other_robot.priority:
            return True
        elif other_robot.priority > self.priority:
            return False
        
        # 6. En caso de empate, el ID más bajo tiene prioridad (determinista)
        return self.unique_id < other_robot.unique_id

    def has_enough_battery_for_path(self):
        """
        Calcula si el robot tiene suficiente batería para completar su ruta actual
        y volver a la estación de carga más cercana si es necesario.
        """
        if not self.path or len(self.path) <= 1:
            return True
        
        nearest_station = self.find_nearest_charging_station()
        if nearest_station:
            distance_to_station = abs(self.pos[0] - nearest_station.pos[0]) + abs(self.pos[1] - nearest_station.pos[1])
            if distance_to_station <= 3:  # Si está a 3 pasos o menos de una estación
                print(f"Robot {self.unique_id}: Estación cercana a {distance_to_station} pasos. Permitiendo movimiento.")
                return True
        
        # Si acaba de salir de carga y tiene más del 90% de batería, permitir continuar
        if hasattr(self, 'just_charged') and self.just_charged:
            if self.battery_level >= self.max_battery * 0.9:
                return True
                
        # Determinar tasa de consumo según modo de energía
        drain_rate = self.energy_saving_drain_rate if self.energy_saving_mode else self.battery_drain_rate
        
        # En rutas cortas (menos de 20 pasos), siempre permitir continuar si tiene más de 40% batería
        steps_left = len(self.path) - 1
        if steps_left < 20 and self.battery_level >= self.max_battery * 0.4:
            return True
        
        # Para rutas medianas (20-40 pasos), necesita al menos 60% de batería
        if steps_left < 40 and self.battery_level >= self.max_battery * 0.6:
            return True
            
        # Para rutas largas, hacer un cálculo preciso
        battery_needed = steps_left * drain_rate
        
        # Margen de seguridad
        safety_margin = self.max_battery * 0.1
        
        # Si tiene un paquete asignado, calcular también la batería necesaria
        # para ir desde el destino hasta la estación de carga más cercana
        extra_margin = 0
        if self.carrying_package and self.package_destination:
            nearest_station = self.find_nearest_charging_station()
            if nearest_station:
                # Calcular distancia estimada (Manhattan) al destino y luego a la estación
                distance_to_station = abs(self.package_destination[0] - nearest_station.pos[0]) + \
                                    abs(self.package_destination[1] - nearest_station.pos[1])
                extra_margin = distance_to_station * drain_rate * 0.5  # Factor 0.5 para no sobreestimar
        
        # Total de batería necesaria
        total_needed = battery_needed + safety_margin + extra_margin
        
        # Imprimir diagnóstico
        battery_percentage = (self.battery_level / self.max_battery) * 100
        total_percentage = (total_needed / self.max_battery) * 100
        print(f"Robot {self.unique_id}: Diagnóstico de batería: {battery_percentage:.1f}% disponible, " +
            f"necesita {total_percentage:.1f}% para {steps_left} pasos " +
            f"(modo ahorro: {self.energy_saving_mode}, tasa: {drain_rate})")
        
        return self.battery_level >= total_needed
    
    def check_state_consistency(self):
        """Verifica y corrige inconsistencias en el estado del robot"""
        # Verificar que self.path no sea None
        if self.path is None:
            print(f"Robot {self.unique_id}: Path es None. Inicializando con posición actual.")
            self.path = [self.pos]
            
        # Si self.path está vacío, inicializarlo con la posición actual
        if len(self.path) == 0:
            print(f"Robot {self.unique_id}: Path está vacío. Inicializando con posición actual.")
            self.path = [self.pos]
            
        # Verificar que el primer elemento de path coincida con la posición actual
        if self.path and self.path[0] != self.pos:
            print(f"Robot {self.unique_id}: Posición actual {self.pos} no coincide con path[0] {self.path[0]}. Corrigiendo.")
            self.path = [self.pos] + self.path
        
        # Verificar estados inconsistentes
        if self.charging and self.idle:
            print(f"Robot {self.unique_id}: No puede estar cargando e idle al mismo tiempo. Corrigiendo.")
            self.idle = False
            
        # Verificar inconsistencias en estados de batería crítica
        if self.critical_battery and self.battery_level > self.max_battery * 0.15:
            print(f"Robot {self.unique_id}: Ya no tiene batería crítica. Reseteando flags.")
            self.critical_battery = False
            self.emergency_route = False
    
    def find_emergency_charging_station(self):
        """Encuentra la estación más cercana en situación de emergencia"""
        if not self.model.charging_stations:
            return None
        
        # Encontrar todas las estaciones ordenadas SOLO por distancia
        stations_by_distance = []
        for station in self.model.charging_stations:
            distance = abs(self.pos[0] - station.pos[0]) + abs(self.pos[1] - station.pos[1])
            # Ignorar ocupación y solo considerar distancia
            stations_by_distance.append((station, distance))
        
        # Ordenar por distancia
        stations_by_distance.sort(key=lambda x: x[1])
        
        if stations_by_distance:
            return stations_by_distance[0][0]  # Retornar la más cercana
        
        return None
    
    def step(self):
        """Método paso del robot reescrito para solucionar problemas de movimiento y carga"""
        if self.charging:
            station = self.is_at_charging_station()
            if not station:
                print(f"Robot {self.unique_id}: ESTADO INCONSISTENTE DETECTADO - En estado de carga pero fuera de estación")
                print(f"Robot {self.unique_id}: Posición actual: {self.pos}, Última posición conocida: {self.last_position}")
                # Detectar robots cercanos que podrían haber causado un desplazamiento
                nearby_robots = [r for r in self.model.robots if r.unique_id != self.unique_id and 
                                abs(r.pos[0] - self.pos[0]) + abs(r.pos[1] - self.pos[1]) <= 2]
                if nearby_robots:
                    print(f"Robot {self.unique_id}: Robots cercanos: {[(r.unique_id, r.pos) for r in nearby_robots]}")
                # Corregir el estado inconsistente
                self.charging = False
                self.nearest_charging_station = None
                # Regresar a la estación más cercana si la batería es baja
                if self.battery_level < self.max_battery * 0.4:
                    nearest_station = self.find_nearest_charging_station()
                    if nearest_station:
                        self.path = self.calculate_path_to_station(nearest_station)
        battery_percentage = self.get_battery_percentage()
    
        # ACCIÓN DE EMERGENCIA para batería crítica
        if battery_percentage <= 10 and not self.charging:
            print(f"Robot {self.unique_id}: ¡BATERÍA CRÍTICA! ({battery_percentage:.1f}%)")
            # Buscar CUALQUIER estación, sin importar colas o distancia
            nearest_station = self.find_emergency_charging_station()
            if nearest_station:
                self.priority = 20  # Máxima prioridad
                # Ruta usando el método de cálculo existente
                self.path = self.calculate_path_to_station(nearest_station)
                print(f"Robot {self.unique_id}: ¡RUTA DE EMERGENCIA ACTIVADA!")

        # Verificar si está en estado idle
        if hasattr(self, 'idle') and self.idle:
            return
        
        self.check_state_consistency()

        # Verificar si está esperando para cargar en una estación
        if self.waiting_for_charge and self.charging_station_target:
            # Si está en la estación, verificar si puede empezar a cargar
            if self.pos == self.charging_station_target.pos:
                if self.handle_charging_station_arrival():
                    # Si está esperando o cargando, terminar el paso
                    return
            else:
                # Si no está en la estación, pero está esperando, verificar si sigue en la cola
                if self.charging_station_target and self.unique_id not in self.charging_station_target.waiting_queue:
                    # Si ya no está en la cola (por algún motivo), resetear el estado
                    self.waiting_for_charge = False
                    self.charging_station_target = None
        
        # Verificar si ha recogido/entregado un paquete en esta posición
        if self.carrying_package and self.pos == self.package_destination:
            if self.check_package_status():
                # Si se recogió/entregó un paquete, resetear el flag returning_to_task y terminar el paso
                self.returning_to_task = False
                return

        # Si ya llegó a la meta y no está cargando, no hacer nada
        if self.reached_goal and not self.charging and not self.waiting_for_charge:
            self.returning_to_task = False
            return
        
        # ===== ACTUALIZAR COOLDOWN POST-CARGA =====
        # Actualizar contador de cooldown
        if hasattr(self, 'just_charged') and self.just_charged:
            if not hasattr(self, 'charge_cooldown'):
                self.charge_cooldown = 0
            self.charge_cooldown += 1
            # Después de 5 pasos, desactivar el flag de "recién cargado"
            if self.charge_cooldown >= 5:
                self.just_charged = False
                self.charge_cooldown = 0
                print(f"Robot {self.unique_id}: Fin de periodo de gracia tras carga")
        
        # ===== VERIFICACIÓN DE BATERÍA SUFICIENTE =====
        # Solo verificar si no está cargando, no está buscando una estación, no está en periodo de gracia
        # y no está esperando para cargar
        if (not self.charging and not self.nearest_charging_station and 
            not (hasattr(self, 'just_charged') and self.just_charged) and 
            not self.waiting_for_charge):
            # Verificar si tiene suficiente batería para la ruta actual
            if not self.has_enough_battery_for_path():
                print(f"Robot {self.unique_id}: Batería insuficiente para completar la ruta ({self.battery_level:.1f}%). Buscando estación de carga...")
                
                # Buscar la estación más adecuada considerando colas
                best_station = self.prioritize_charging_stations()
                
                if best_station:
                    # Guardar el camino original si aún no se ha llegado a la meta
                    if not self.reached_goal:
                        self.original_path = self.path.copy() if self.path else []
                        
                    # Añadirse a la cola previamente - Verificar que la operación sea exitosa
                    add_success = best_station.add_to_queue(self.unique_id)
                    
                    # Establecer la estación como destino
                    self.nearest_charging_station = best_station
                    self.charging_station_target = best_station
                    self.path = self.calculate_path_to_station(best_station)
                    
                    if self.path:
                        if add_success:
                            # Solo intentar mostrar la posición en cola si se añadió correctamente
                            if self.unique_id in best_station.waiting_queue:
                                queue_pos = best_station.waiting_queue.index(self.unique_id) + 1
                                print(f"Robot {self.unique_id}: Redirigiendo a estación en {best_station.pos}. Nueva ruta calculada. Posición en cola: {queue_pos}")
                            else:
                                print(f"Robot {self.unique_id}: Redirigiendo a estación en {best_station.pos}. Nueva ruta calculada.")
                        else:
                            print(f"Robot {self.unique_id}: Redirigiendo a estación en {best_station.pos}. Nueva ruta calculada.")
                    else:
                        # Si no se puede encontrar ruta, eliminar de la cola
                        if self.unique_id in best_station.waiting_queue:
                            best_station.remove_from_queue(self.unique_id)
                        self.nearest_charging_station = None
                        self.charging_station_target = None
                        print(f"Robot {self.unique_id}: No se pudo encontrar ruta a la estación de carga.")
                else:
                    print(f"Robot {self.unique_id}: No hay estaciones de carga disponibles.")
        
        # ===== MANEJO DE ESTACIÓN DE CARGA =====
        if self.charging:
            # Verificar si sigue en la estación
            station = self.is_at_charging_station()
            
            if station:
                # Cargar batería
                self.charge_battery(station.charging_rate)
                
                # Si la batería está completa, preparar para continuar
                if self.battery_level >= self.max_battery * 0.95:  # 95% de carga
                    print(f"Robot {self.unique_id}: Batería cargada al {self.get_battery_percentage():.1f}%. Preparando para continuar.")
                    
                    # Notificar a la estación que terminamos de cargar
                    if station.finish_charging(self.unique_id):
                        print(f"Robot {self.unique_id}: Liberando estación para el siguiente robot.")
                    
                    # Importante: Primero establecer variables de estado antes de recalcular rutas
                    self.charging = False
                    self.nearest_charging_station = None
                    self.returning_to_task = True
                    self.idle = False  # Asegurar que no esté en idle
                    self.critical_battery = False  # Resetear el flag de batería crítica
                    self.emergency_route = False   # Resetear el flag de ruta de emergencia
                    
                    # Activar flag de "recién cargado" para prevenir bucles
                    if not hasattr(self, 'just_charged'):
                        self.just_charged = False
                        self.charge_cooldown = 0
                    self.just_charged = True
                    self.charge_cooldown = 0
                    
                    # Determinar el destino correcto basado en el estado del paquete
                    destino = None
                    if self.carrying_package:
                        if self.carrying_package.status == 'assigned':
                            destino = self.carrying_package.pickup_location
                            self.package_destination = destino
                            print(f"Robot {self.unique_id}: Retomando ruta hacia punto de recogida del paquete {self.carrying_package.id}.")
                        elif self.carrying_package.status == 'picked':
                            destino = self.carrying_package.delivery_location
                            self.package_destination = destino
                            print(f"Robot {self.unique_id}: Retomando ruta hacia punto de entrega del paquete {self.carrying_package.id}.")
                    else:
                        destino = self.goal
                        print(f"Robot {self.unique_id}: Retomando ruta hacia meta original {self.goal}.")
                    
                    # Forzar el cálculo de una nueva ruta viable
                    if destino:
                        # Método 1: Usar A* directamente
                        nueva_ruta = self.astar(self.pos, destino)
                        
                        # Si no funciona, intentar con penalización
                        if not nueva_ruta or len(nueva_ruta) <= 1:
                            print(f"Robot {self.unique_id}: Intentando ruta con penalización")
                            nueva_ruta = self.astar_with_robot_penalty(self.pos, destino)
                        
                        # Como último recurso, intentar con desvío
                        if not nueva_ruta or len(nueva_ruta) <= 1:
                            print(f"Robot {self.unique_id}: Intentando ruta con desvío")
                            nueva_ruta = self.find_path_with_detour(self.pos, destino)
                        
                        if nueva_ruta and len(nueva_ruta) > 1:
                            self.path = nueva_ruta
                            print(f"Robot {self.unique_id}: RUTA ENCONTRADA con {len(nueva_ruta)} pasos. Primer paso: {nueva_ruta[1]}")
                            
                            # FORZAR MOVIMIENTO INMEDIATO para evitar atascos
                            next_pos = self.path[1]  # El siguiente paso en la ruta
                            
                            # Verificar si el siguiente paso está libre
                            blocking_robot = None
                            for robot in self.model.robots:
                                if robot.unique_id != self.unique_id and robot.pos == next_pos:
                                    blocking_robot = robot
                                    break
                            
                            if blocking_robot is None:
                                # El camino está libre, mover inmediatamente
                                print(f"Robot {self.unique_id}: MOVIMIENTO FORZADO después de cargar a {next_pos}")
                                self.path.pop(0)  # Eliminar posición actual
                                self.model.grid.move_agent(self, next_pos)
                                self.steps_taken += 1
                                self.drain_battery()  # Consumir batería por el movimiento
                                self.last_position = next_pos  # Actualizar última posición
                                self.position_unchanged_count = 0  # Resetear contador de posición sin cambios
                                return  # Terminar el paso después del movimiento forzado
                            else:
                                print(f"Robot {self.unique_id}: Movimiento bloqueado por Robot {blocking_robot.unique_id}")
                                print(f"Robot {self.unique_id}: Movimiento bloqueado después de cargar. Buscando ruta alternativa.")
                                # Incrementar la prioridad para próximos intentos
                                self.priority += 2
                                # Buscar una ruta alternativa inmediatamente
                                self.find_alternative_route()
                                # Si aún así no se puede mover, esperar algunos pasos y resetear estado
                                if not self.path or len(self.path) <= 1:
                                    self.waiting_time = 3  # Preparar para intentar alternativas pronto
                        else:
                            print(f"Robot {self.unique_id}: ERROR CRÍTICO - No se pudo calcular ruta después de cargar")
                            # Como medida extrema, hacer que el robot sea idle para que pueda recibir nuevas tareas
                            self.path = [self.pos]
                            self.idle = True
                            
                            # Si tiene un paquete asignado pero no recogido, liberarlo
                            if self.carrying_package and self.carrying_package.status == 'assigned':
                                print(f"Robot {self.unique_id}: Liberando paquete {self.carrying_package.id} por imposibilidad de movimiento")
                                self.carrying_package.status = 'waiting'
                                self.carrying_package.assigned_robot_id = None
                                self.carrying_package = None
                    else:
                        print(f"Robot {self.unique_id}: No se pudo determinar un destino válido")
                        self.idle = True
                    
                    return  # Importante: terminar el paso después de preparar la ruta
            else:
                # Si ya no está en estación pero estaba cargando, reiniciar estado
                print(f"Robot {self.unique_id}: ERROR - Ya no está en estación de carga")
                self.charging = False
                self.nearest_charging_station = None
                # Reiniciar path con posición actual
                self.path = [self.pos]
                return  # Terminar este paso para permitir recálculo en el siguiente

        # ===== DETECCIÓN DE BLOQUEO =====
        # Actualizar contador de posición sin cambios
        if self.last_position == self.pos:
            self.position_unchanged_count += 1
        else:
            self.position_unchanged_count = 0
            self.last_position = self.pos
        # ===== MANEJO ESPECIAL DE BLOQUEOS CERCA DE ESTACIONES DE CARGA =====
        # Verificar si estamos bloqueados cerca de una estación de carga
        if self.handle_charging_station_blocking():
            return  # Si se tomó alguna acción, terminar el paso

        
    
        # Si lleva demasiado tiempo sin moverse, intentar soluciones
        if self.position_unchanged_count > 5:
            self.priority += 1
            print(f"Robot {self.unique_id}: Posiblemente bloqueado por {self.position_unchanged_count} pasos, aumentando prioridad a {self.priority}")
            
            # En bloqueos severos, buscar alternativas más drásticas
            if self.position_unchanged_count > 10:
                print(f"Robot {self.unique_id}: Bloqueo prolongado, buscando alternativas...")
                self.find_alternative_route()
                
                # Para bloqueos muy prolongados, resetear estado
                if self.position_unchanged_count > 20:
                    print(f"Robot {self.unique_id}: BLOQUEO CRÍTICO, RESETEANDO COMPLETAMENTE")
                    # Reiniciar todos los estados relacionados con movimiento
                    self.returning_to_task = False
                    self.waiting_for_charge = False
                    self.charging = False
                    self.nearest_charging_station = None
                    self.charging_station_target = None
                    self.critical_battery = False
                    
                    # Liberar recursos y reiniciar
                    if self.carrying_package and self.carrying_package.status == 'assigned':
                        self.carrying_package.status = 'waiting'
                        self.carrying_package.assigned_robot_id = None
                        self.carrying_package = None
                    
                    # Establecer en idle para recibir nuevas tareas
                    self.idle = True
                    self.path = [self.pos]
                    
                    # Resetear contadores de bloqueo
                    self.position_unchanged_count = 0
                    self.blocked_count = 0
                    self.waiting_time = 0
                    
                    print(f"Robot {self.unique_id}: Estado completamente reseteado. Esperando nueva tarea.")
                    return

        # ===== MOVIMIENTO NORMAL =====
        if len(self.path) > 1:  # Verificar que hay al menos un paso más en la ruta
            # Verificar si hay suficiente batería para moverse
            if not self.drain_battery():
                return  # Batería agotada, no moverse
            
            if len(self.path) <= 1:  # Verificación extra por si la ruta cambió
                print(f"Robot {self.unique_id}: Ruta demasiado corta después de drenar batería")
                return
                
            next_pos = self.path[1]  # El siguiente paso en la ruta
            
            # DEBUG: Imprimir información de movimiento
            print(f"Robot {self.unique_id}: Intentando moverse de {self.pos} a {next_pos}")
            
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
                
                self.path.pop(0)  # Eliminar posición actual de la ruta
                self.model.grid.move_agent(self, next_pos)
                self.steps_taken += 1
                
                # Verificar si llegó a una estación de carga
                station = self.is_at_charging_station()
                if station and self.nearest_charging_station and not self.charging:
                    if self.handle_charging_station_arrival():
                        return
                
                if station and self.nearest_charging_station:
                    print(f"Robot {self.unique_id}: Llegó a estación de carga.")
                    self.charging = True
                elif self.pos == self.goal and not self.returning_to_task:
                    self.reached_goal = True
                    print(f"¡Robot {self.unique_id} ha alcanzado el objetivo! (Batería: {self.battery_level:.1f}%)")
                elif self.pos == self.goal and self.returning_to_task:
                    # Ha alcanzado el objetivo mientras retornaba de carga
                    print(f"Robot {self.unique_id}: Ha llegado al objetivo tras retornar de carga")
                    self.returning_to_task = False
                    
                    # Verificar si tiene un paquete y está en el lugar correcto
                    if self.carrying_package and self.pos == self.package_destination:
                        self.check_package_status()
                else:
                    status = "Cargando" if self.charging else "Retornando" if self.returning_to_task else "Normal"
                    print(f"Robot {self.unique_id} se movió a {next_pos} (Paso {self.steps_taken}, Batería: {self.battery_level:.1f}%, Estado: {status})")
            else:
                # Camino bloqueado por otro robot
                self.blocked_count += 1
                print(f"Robot {self.unique_id}: Bloqueado por Robot {blocking_robot.unique_id} (intento {self.blocked_count})")
                
                # Determinar qué robot tiene mayor prioridad usando el nuevo sistema de prioridades
                # 1. Los robots en estado crítico de batería tienen la máxima prioridad
                # 2. Los robots que van a una estación de carga con batería baja (<=20%) tienen prioridad
                # 3. Entre robots de prioridad similar, el de menor batería porcentual tiene prioridad
                # 4. Si todavía hay empate, comparar prioridad numérica, luego ID más bajo
                
                has_critical_battery = self.critical_battery or self.battery_level < self.max_battery * 0.08
                other_has_critical = hasattr(blocking_robot, 'critical_battery') and blocking_robot.critical_battery
                
                going_to_charge = self.nearest_charging_station is not None and self.battery_level < self.max_battery * 0.2
                other_going_to_charge = (hasattr(blocking_robot, 'nearest_charging_station') and 
                                        blocking_robot.nearest_charging_station is not None and 
                                        blocking_robot.battery_level < blocking_robot.max_battery * 0.2)
                
                if has_critical_battery and not other_has_critical:
                    # Este robot tiene prioridad por batería crítica
                    has_priority = True
                elif not has_critical_battery and other_has_critical:
                    # El otro robot tiene prioridad por batería crítica
                    has_priority = False
                elif going_to_charge and not other_going_to_charge:
                    # Este robot tiene prioridad por ir a cargar con batería baja
                    has_priority = True
                elif not going_to_charge and other_going_to_charge:
                    # El otro robot tiene prioridad por ir a cargar con batería baja
                    has_priority = False
                elif self.nearest_charging_station and blocking_robot.nearest_charging_station:
                    # Ambos van a cargar - el que tenga menos batería % tiene prioridad
                    self_percentage = self.battery_level / self.max_battery
                    other_percentage = blocking_robot.battery_level / blocking_robot.max_battery
                    if self_percentage < other_percentage:
                        has_priority = True
                    elif other_percentage < self_percentage:
                        has_priority = False
                    else:
                        # Si tienen la misma proporción de batería, usar prioridad numérica
                        if self.priority > blocking_robot.priority:
                            has_priority = True
                        elif blocking_robot.priority > self.priority:
                            has_priority = False
                        else:
                            # Usar ID como desempate final
                            has_priority = self.unique_id < blocking_robot.unique_id
                else:
                    # Ninguno va a cargar - seguir con las reglas estándar
                    if self.priority > blocking_robot.priority:
                        has_priority = True
                    elif blocking_robot.priority > self.priority:
                        has_priority = False
                    else:
                        # Usar ID como desempate final
                        has_priority = self.unique_id < blocking_robot.unique_id
                
                # Actuar según la determinación de prioridad
                if has_priority:
                    # Este robot tiene mayor prioridad
                    print(f"Robot {self.unique_id} (batería: {self.battery_level:.1f}%, prioridad: {self.priority}) tiene prioridad sobre Robot {blocking_robot.unique_id} (batería: {blocking_robot.battery_level:.1f}%, prioridad: {blocking_robot.priority})")
                    
                    # Si está bloqueado por poco tiempo, esperar
                    if self.blocked_count < 3:
                        print(f"Robot {self.unique_id} esperando {self.blocked_count} turno(s)...")
                        return
                    
                    # Después de esperar, buscar ruta alternativa
                    self.find_alternative_route()
                else:
                    # El otro robot tiene mayor prioridad
                    print(f"Robot {self.unique_id} cede el paso a Robot {blocking_robot.unique_id} que tiene mayor prioridad")
                    
                    # Si tiene batería crítica, intentar rutas alternativas inmediatamente
                    if has_critical_battery:
                        print(f"Robot {self.unique_id}: Batería crítica/baja, buscando ruta alternativa urgente")
                        self.find_alternative_route()
                        return
                        
                    # En otros casos, esperar un poco y luego buscar alternativas
                    self.waiting_time += 1
                    if self.waiting_time > 2:
                        print(f"Robot {self.unique_id} ha esperado {self.waiting_time} turnos, buscando ruta alternativa")
                        self.find_alternative_route()
        elif len(self.path) == 1 and self.pos == self.path[0]:
            # Si llegó al final de la ruta
            if self.pos == self.goal:
                if not self.returning_to_task:
                    self.reached_goal = True
                    print(f"¡Robot {self.unique_id} ha alcanzado el objetivo final! (Batería: {self.battery_level:.1f}%)")
                else:
                    # Ha llegado al objetivo mientras retornaba de una tarea
                    self.returning_to_task = False
                    print(f"Robot {self.unique_id} ha completado su retorno tras cargar.")
                    
                    # Verificar si tiene un paquete y está en el lugar correcto
                    if self.carrying_package and self.pos == self.package_destination:
                        self.check_package_status()
            
            # Verificar si está en una estación de carga
            station = self.is_at_charging_station()
            
            if station and not self.charging and self.nearest_charging_station:
                print(f"Robot {self.unique_id}: Llegó a estación de carga.")
                self.charging = True
        else:
            # Si no tiene ruta o es inválida, imprimir advertencia
            print(f"Robot {self.unique_id}: ADVERTENCIA - No tiene una ruta válida. Path actual: {self.path}")
            if not self.charging and not self.nearest_charging_station:
                print(f"Robot {self.unique_id}: Reseteando a estado idle por falta de ruta válida")
                self.idle = True
    
    def find_alternative_route(self):
        """Busca una ruta alternativa cuando el robot está bloqueado"""
        print(f"Robot {self.unique_id}: Buscando ruta alternativa...")
        
        # Guardar la ruta actual para compararla
        old_path = self.path.copy() if self.path else []
        
        # Determinar destino final (meta del robot o estación de carga)
        destination = None
        if self.charging or self.nearest_charging_station:
            # Si tiene batería crítica, reevaluar qué estación es la mejor en este momento
            if self.critical_battery or self.battery_level < self.max_battery * 0.15:
                best_station = self.prioritize_charging_stations()
                if best_station:
                    # Solo cambiar de destino si encontramos una estación mejor
                    self.nearest_charging_station = best_station
                    self.charging_station_target = best_station
                    destination = best_station.pos
                    print(f"Robot {self.unique_id}: Reevaluando estaciones debido a batería baja. Nueva estación: {destination}")
                else:
                    # Si no hay mejor estación, mantener la actual
                    destination = self.nearest_charging_station.pos
            else:
                # Si no es crítico, mantener el destino original
                destination = self.nearest_charging_station.pos
        elif self.carrying_package:
            if self.carrying_package.status == 'assigned':
                destination = self.carrying_package.pickup_location
            elif self.carrying_package.status == 'picked':
                destination = self.carrying_package.delivery_location
        else:
            destination = self.goal
        
        if not destination:
            print(f"Robot {self.unique_id}: No se pudo determinar destino para ruta alternativa.")
            return False
            
        # Para robots con batería crítica, usar métodos más agresivos primero
        if self.critical_battery or self.battery_level < self.max_battery * 0.15:
            # Primero intentar con penalización fuerte para alejarse de otros robots
            print(f"Robot {self.unique_id}: Buscando ruta prioritaria por batería crítica...")
            self.path = self.astar_with_robot_penalty(self.pos, destination, penalty_multiplier=2.0)
            
            # Si no funciona, intentar con desvío
            if not self.path or len(self.path) < 2:
                self.path = self.find_path_with_detour(self.pos, destination)
                
            # Si aún no funciona, intentar A* normal
            if not self.path or len(self.path) < 2:
                self.path = self.astar(self.pos, destination)
        else:
            # Estrategia normal para robots sin problemas de batería
            # Estrategia 1: Ruta normal A* (ya intentada pero volvemos a intentar por si hay cambios)
            self.path = self.astar(self.pos, destination)
            
            # Si no encontró ruta o es la misma, probar con penalización de posiciones ocupadas
            if not self.path or len(self.path) < 2 or self.path == old_path or self.path_in_tried_alternatives(self.path):
                print(f"Robot {self.unique_id}: Buscando ruta con penalización de robots...")
                self.path = self.astar_with_robot_penalty(self.pos, destination)
            
            # Si aún no encuentra ruta, intentar una ruta más larga con desvío
            if not self.path or len(self.path) < 2 or self.path == old_path or self.path_in_tried_alternatives(self.path):
                print(f"Robot {self.unique_id}: Buscando ruta con desvío...")
                self.path = self.find_path_with_detour(self.pos, destination)
        
        # Desvío aleatorio como último recurso (para ambos casos)
        if not self.path or len(self.path) < 2 or self.path == old_path or self.path_in_tried_alternatives(self.path):
            print(f"Robot {self.unique_id}: Intentando desvío aleatorio...")
            import random
            
            # Si tiene batería crítica, buscar puntos cercanos a estaciones conocidas
            potential_points = []
            if self.critical_battery and self.model.charging_stations:
                for station in self.model.charging_stations:
                    # Puntos alrededor de la estación
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                        potential_points.append((station.pos[0] + dx, station.pos[1] + dy))
            
            # Generar puntos aleatorios como respaldo
            for _ in range(5):
                random_x = random.randint(0, self.model.grid.width - 1)
                random_y = random.randint(0, self.model.grid.height - 1)
                potential_points.append((random_x, random_y))
            
            # Filtrar puntos válidos
            valid_points = []
            for point in potential_points:
                if (0 <= point[0] < self.model.grid.width and 
                    0 <= point[1] < self.model.grid.height and
                    not self.model.has_obstacle(point)):
                    # Añadir distancia para ordenar
                    distance = abs(self.pos[0] - point[0]) + abs(self.pos[1] - point[1])
                    valid_points.append((point, distance))
            
            # Ordenar por distancia (los puntos más cercanos primero)
            valid_points.sort(key=lambda x: x[1])
            
            # Intentar encontrar ruta con cada punto
            for punto_desvio, _ in valid_points:
                # Buscar ruta hasta este punto
                ruta_a_desvio = self.astar(self.pos, punto_desvio)
                if ruta_a_desvio and len(ruta_a_desvio) > 1:
                    # Para batería crítica, solo nos importa llegar al punto de desvío
                    # que podría estar cerca de una estación
                    if self.critical_battery:
                        self.path = ruta_a_desvio
                        print(f"Robot {self.unique_id}: Ruta de emergencia hacia {punto_desvio}")
                        break
                    # Para casos normales, buscar ruta completa
                    else:
                        # Luego buscar ruta desde ahí hasta el destino final
                        ruta_desde_desvio = self.astar(punto_desvio, destination)
                        if ruta_desde_desvio and len(ruta_desde_desvio) > 1:
                            # Combinar las rutas (eliminar duplicado del punto de desvío)
                            self.path = ruta_a_desvio + ruta_desde_desvio[1:]
                            print(f"Robot {self.unique_id}: Ruta con desvío aleatorio encontrada a través de {punto_desvio}")
                            break
        
        # Si sigue sin encontrar ruta, esperar y mantener la ruta anterior
        if not self.path or len(self.path) < 2:
            print(f"Robot {self.unique_id}: No se encontró ruta alternativa, manteniendo la actual y esperando...")
            self.path = old_path if old_path else [self.pos]
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
    
    def find_alternative_charging_station(self, excluding=None):
        """Busca una estación de carga alternativa, excluyendo las especificadas"""
        # Limpiar el estado actual de espera si vamos a buscar otra estación
        if self.waiting_for_charge and self.current_charging_station:
            # Eliminar de la cola de la estación actual antes de buscar otra
            self.current_charging_station.remove_from_queue(self.unique_id)
            self.waiting_for_charge = False
            self.current_charging_station = None
        
        # Resto del código existente para buscar una estación alternativa...
        if not self.model.charging_stations:
            return None
        
        # Calcular para cada estación: distancia, ocupación y si podemos llegar
        stations_info = []
        for station in self.model.charging_stations:
            # Excluir la estación especificada
            if excluding and station.pos == excluding:
                continue
                
            # Calcular distancia Manhattan
            distance = abs(self.pos[0] - station.pos[0]) + abs(self.pos[1] - station.pos[1])
            
            # Verificar si tenemos batería para llegar
            battery_needed = distance * self.battery_drain_rate * 1.1  # 10% margen
            can_reach = self.battery_level >= battery_needed
            
            # Calcular ocupación (robots cargando + en cola)
            occupation = len(station.waiting_queue) + (1 if station.current_robot else 0)
            
            stations_info.append((station, distance, can_reach, occupation, battery_needed))
        
        # Si hay estaciones a las que podemos llegar, ordenar por (ocupación, distancia)
        reachable = [s for s in stations_info if s[2]]
        if reachable:
            reachable.sort(key=lambda x: (x[3], x[1]))  # Ordenar por ocupación y luego distancia
            best_station = reachable[0][0]
            print(f"Robot {self.unique_id}: Estación alternativa en {best_station.pos}, ocupación: {reachable[0][3]}, distancia: {reachable[0][1]}, batería requerida: {reachable[0][4]:.1f}")
            return best_station
        
        # Si no hay reachable, intentar con la más cercana si es una emergencia
        if self.battery_level < self.max_battery * 0.08 and stations_info:
            stations_info.sort(key=lambda x: x[1])  # Ordenar solo por distancia
            print(f"Robot {self.unique_id}: Intentando llegar a estación más cercana como emergencia")
            return stations_info[0][0]
        
        return None

    def handle_charging_station_blocking(self):
        """
        Verifica si el robot está bloqueado cerca de una estación de carga y toma medidas
        para resolver la situación.
        
        Returns:
            bool: True si se tomó alguna acción, False en otro caso
        """
        # Solo aplicar esta lógica a robots que están intentando llegar a una estación
        if not self.nearest_charging_station or not self.charging_station_target:
            return False
            
        # Verificar si estamos cerca de la estación objetivo (a 1-3 pasos)
        station_pos = self.charging_station_target.pos
        distance_to_station = abs(self.pos[0] - station_pos[0]) + abs(self.pos[1] - station_pos[1])
        
        # Si estamos cerca pero llevamos varios turnos sin movernos
        if 1 <= distance_to_station <= 3 and self.position_unchanged_count >= 3:
            print(f"Robot {self.unique_id}: Detectado bloqueo cerca de estación {station_pos} ({distance_to_station} pasos)")
            
            # Determinar si hay otros robots en la estación o cerca de ella
            robots_at_station = False
            for robot in self.model.robots:
                if robot.unique_id != self.unique_id and robot.pos == station_pos:
                    robots_at_station = True
                    print(f"Robot {self.unique_id}: Estación ocupada por Robot {robot.unique_id}")
                    break
            
            # Si llevamos esperando demasiado tiempo o la batería está crítica, buscar otra estación
            if self.position_unchanged_count > 5 or self.battery_level < self.max_battery * 0.1:
                print(f"Robot {self.unique_id}: Bloqueado por demasiado tiempo o batería crítica. Buscando alternativas.")
                
                # Quitar el robot de la cola de la estación actual
                if self.charging_station_target.remove_from_queue(self.unique_id):
                    print(f"Robot {self.unique_id}: Abandonando cola en estación {station_pos}")
                
                # Reiniciar el estado de carga
                self.nearest_charging_station = None
                self.charging_station_target = None
                self.waiting_for_charge = False
                
                # Buscar una estación alternativa, excluyendo la actual
                alternative_station = self.find_alternative_charging_station(excluding=station_pos)
                
                if alternative_station:
                    print(f"Robot {self.unique_id}: Encontrada estación alternativa en {alternative_station.pos}")
                    
                    # Establecer nueva estación objetivo
                    self.nearest_charging_station = alternative_station
                    self.charging_station_target = alternative_station
                    
                    # Añadir a la cola de la nueva estación
                    alternative_station.add_to_queue(self.unique_id)
                    
                    # Calcular nueva ruta
                    self.path = self.calculate_path_to_station(alternative_station)
                    
                    if self.path and len(self.path) > 1:
                        print(f"Robot {self.unique_id}: Redirigiendo a estación alternativa ({len(self.path)} pasos)")
                        return True
                    else:
                        print(f"Robot {self.unique_id}: No se pudo calcular ruta a estación alternativa")
                else:
                    print(f"Robot {self.unique_id}: No se encontraron estaciones alternativas")
                    
                    # Como último recurso, si la batería es muy crítica, intentar llegar a la estación original
                    # desde otra dirección
                    if self.battery_level < self.max_battery * 0.08:
                        # Buscar puntos cercanos a la estación y accesibles
                        possible_points = []
                        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                            point = (station_pos[0] + dx, station_pos[1] + dy)
                            if 0 <= point[0] < self.model.grid.width and 0 <= point[1] < self.model.grid.height:
                                if not self.model.has_obstacle(point):
                                    # Verificar si hay un robot en esta posición
                                    has_robot = False
                                    for robot in self.model.robots:
                                        if robot.unique_id != self.unique_id and robot.pos == point:
                                            has_robot = True
                                            break
                                    
                                    if not has_robot:
                                        # Calcular distancia desde nuestra posición
                                        dist = abs(self.pos[0] - point[0]) + abs(self.pos[1] - point[1])
                                        # Solo considerar puntos que no están en nuestra dirección actual
                                        if abs(self.pos[0] - point[0]) != distance_to_station and abs(self.pos[1] - point[1]) != distance_to_station:
                                            possible_points.append((point, dist))
                        
                        # Ordenar por distancia
                        possible_points.sort(key=lambda x: x[1])
                        
                        # Intentar llegar al punto más cercano
                        if possible_points:
                            alt_point = possible_points[0][0]
                            alt_path = self.astar(self.pos, alt_point)
                            
                            if alt_path and len(alt_path) > 1:
                                self.path = alt_path
                                print(f"Robot {self.unique_id}: RUTA DE EMERGENCIA a punto alternativo {alt_point} cerca de estación")
                                
                                # Volver a añadir a la cola
                                self.charging_station_target = self.nearest_charging_station = self.find_nearest_charging_station()
                                if self.charging_station_target:
                                    self.charging_station_target.add_to_queue(self.unique_id)
                                
                                return True
                    
                    # Si todo lo demás falla, buscar una ruta alternativa genérica
                    self.find_alternative_route()
                    return True
                    
            # Si la estación está ocupada pero no hemos esperado demasiado, seguir esperando
            elif robots_at_station:
                # Si no estamos en la cola, intentar añadirnos
                if self.unique_id not in self.charging_station_target.waiting_queue:
                    self.charging_station_target.add_to_queue(self.unique_id)
                    print(f"Robot {self.unique_id}: Añadido a cola de espera de estación {station_pos}")
                
                # Si la batería es muy baja, intentar una ruta alternativa
                if self.battery_level < self.max_battery * 0.1:
                    print(f"Robot {self.unique_id}: Batería muy baja mientras espera, intentando otra ruta")
                    self.find_alternative_route()
                else:
                    print(f"Robot {self.unique_id}: Esperando en cola para estación {station_pos}")
                    
                return True
        
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

    def astar_with_robot_penalty(self, start, goal, penalty_multiplier=1.0):
        """A* con penalización adicional por celdas cercanas a robots"""
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        # Crear un mapa de penalizaciones basado en posiciones de robots
        robot_penalty_map = {}
        for robot in self.model.robots:
            if robot.unique_id != self.unique_id:
                # Penalizar la posición del robot
                robot_penalty_map[robot.pos] = 10 * penalty_multiplier
                
                # Penalizar posiciones adyacentes
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    pos = (robot.pos[0] + dx, robot.pos[1] + dy)
                    if 0 <= pos[0] < self.model.grid.width and 0 <= pos[1] < self.model.grid.height:
                        robot_penalty_map[pos] = robot_penalty_map.get(pos, 0) + 5 * penalty_multiplier
        
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
            battery_drain_rate = config.get('battery_drain_rate', 0.5)
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
    
    # AÑADIR ESTE MÉTODO A LA CLASE PathFindingModel
    def check_robots_health(self):
        """Verifica periódicamente el estado de todos los robots"""
        for robot in self.robots:
            # Detectar robots con batería crítica no dirigiéndose a cargar
            if robot.battery_level < robot.max_battery * 0.15 and not robot.charging and not robot.nearest_charging_station:
                print(f"Robot {robot.unique_id}: ALERTA DE SISTEMA - Batería crítica.")
                # Forzar búsqueda de estación
                nearest_station = robot.find_emergency_charging_station()
                if nearest_station:
                    robot.nearest_charging_station = nearest_station
                    robot.charging_station_target = nearest_station
                    robot.path = robot.calculate_path_to_station(nearest_station)
            
            # Detectar robots atascados
            if hasattr(robot, 'position_unchanged_count') and robot.position_unchanged_count > 10:
                print(f"Robot {robot.unique_id}: ATENCIÓN - Robot atascado por {robot.position_unchanged_count} pasos.")
                # Forzar reseteo de estado
                if hasattr(robot, 'find_alternative_route'):
                    robot.find_alternative_route()
                # Aumentar prioridad dramáticamente
                if hasattr(robot, 'priority'):
                    robot.priority += 5
    
    def step(self):
        self.check_robots_health()
        self.datacollector.collect(self)
        self.schedule.step()
    
    def all_robots_reached_goal(self):
        """Verifica si todos los robots han alcanzado su meta"""
        return all(robot.reached_goal for robot in self.robots)