from mesa import Agent, Model
from mesa.space import MultiGrid
from mesa.time import BaseScheduler
from mesa.datacollection import DataCollector

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
        self.returning_to_task = False
        
        if not self.path:
            print(f"Robot {unique_id}: No se encontró camino del inicio al objetivo.")
        else:
            print(f"Robot {unique_id}: Ruta calculada: {self.path}")
            print(f"Robot {unique_id}: Nivel de batería: {self.battery_level}%")
    
    def astar(self, start, goal):
        def heuristic(a, b):
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
            for d in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                neighbor = (current[0] + d[0], current[1] + d[1])
                if 0 <= neighbor[0] < self.model.grid.width and 0 <= neighbor[1] < self.model.grid.height:
                    # Verificar si hay obstáculos
                    if not self.model.has_obstacle(neighbor):
                        # Verificar si hay otros robots en la posición
                        # Solo considerar como bloqueado si el robot no está en movimiento
                        # (para permitir que los robots se muevan a través de la ruta planificada)
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
        
        # Movimiento normal (si no está cargando)
        elif len(self.path) > 1:
            # Verificar si hay suficiente batería para moverse
            if not self.drain_battery():
                return  # Batería agotada, no moverse
                
            next_pos = self.path[1]  # El siguiente paso en la ruta
            
            # Verificar si el siguiente paso está ocupado por otro robot
            occupied = False
            for robot in self.model.robots:
                if robot.unique_id != self.unique_id and robot.pos == next_pos:
                    occupied = True
                    break
            
            if not occupied:
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
                # Recalcular ruta si hay colisión
                old_path = self.path
                self.path = self.astar(self.pos, self.path[-1])  # Recalcular a la meta actual (podría ser estación o meta original)
                if self.path != old_path:
                    print(f"Robot {self.unique_id} recalculó su ruta debido a una colisión.")
                    
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

class PathFindingModel(Model):
    def __init__(self, width, height, robot_configs, charging_station_positions=None):
        super().__init__()
        self.grid = MultiGrid(width, height, torus=False)
        self.schedule = BaseScheduler(self)
        self.obstacles = []  # Lista para almacenar los agentes obstáculo
        self.robots = []  # Lista para almacenar los robots
        self.charging_stations = []  # Lista para almacenar las estaciones de carga
        
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