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

class RobotAgent(Agent):
    def __init__(self, unique_id, model, start, goal, color="red"):
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
        
        if not self.path:
            print(f"Robot {unique_id}: No se encontró camino del inicio al objetivo.")
        else:
            print(f"Robot {unique_id}: Ruta calculada: {self.path}")
    
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

    def step(self):
        if self.reached_goal:
            return  # No hacer nada si ya alcanzó la meta
            
        if len(self.path) > 1:
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
                print(f"Robot {self.unique_id} se movió a {next_pos} (Paso {self.steps_taken})")
            else:
                # Recalcular ruta si hay colisión
                self.path = self.astar(self.pos, self.goal)
                print(f"Robot {self.unique_id} recalculó su ruta debido a una colisión")
                
        elif len(self.path) == 1 and self.pos == self.goal:
            self.reached_goal = True
            print(f"¡Robot {self.unique_id} ha alcanzado el objetivo!")

class PathFindingModel(Model):
    def __init__(self, width, height, robot_configs):
        super().__init__()
        self.grid = MultiGrid(width, height, torus=False)
        self.schedule = BaseScheduler(self)
        self.obstacles = []  # Lista para almacenar los agentes obstáculo
        self.robots = []  # Lista para almacenar los robots
        
        # Crear y colocar los robots
        robot_id = 1
        for config in robot_configs:
            # Extraer configuración del robot
            start = config['start']
            goal = config['goal']
            color = config.get('color', "red")  # Color por defecto
            
            robot = RobotAgent(robot_id, self, start, goal, color)
            self.robots.append(robot)
            self.schedule.add(robot)
            self.grid.place_agent(robot, tuple(start) if isinstance(start, list) else start)
            robot_id += 1
        
        # Añadir recolector de datos para estadísticas
        self.datacollector = DataCollector(
            model_reporters={
                "Pasos Promedio": lambda m: sum(robot.steps_taken for robot in m.robots) / len(m.robots) if m.robots else 0,
                "Robots en Meta": lambda m: sum(1 for robot in m.robots if robot.reached_goal)
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
                    robot.path = robot.astar(robot.pos, robot.goal)
            
            return True
        
        return False
    
    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()
    
    def all_robots_reached_goal(self):
        """Verifica si todos los robots han alcanzado su meta"""
        return all(robot.reached_goal for robot in self.robots)