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
    def __init__(self, unique_id, model, start, goal):
        super().__init__(unique_id, model)
        self.start = start
        self.goal = goal
        self.path = self.astar(start, goal)
        self.steps_taken = 0
        
        if not self.path:
            print("No se encontró camino del inicio al objetivo.")
        else:
            print(f"Ruta calculada: {self.path}")
    
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
        if len(self.path) > 1:
            self.path.pop(0)
            next_pos = self.path[0]
            self.model.grid.move_agent(self, next_pos)
            self.steps_taken += 1
            print(f"El robot se movió a {next_pos} (Paso {self.steps_taken})")
        else:
            print("¡El robot ha alcanzado el objetivo!")

class PathFindingModel(Model):
    def __init__(self, width, height, start, goal):
        super().__init__()
        self.grid = MultiGrid(width, height, torus=False)
        self.schedule = BaseScheduler(self)
        self.start = start
        self.goal = goal
        self.obstacles = []  # Lista para almacenar los agentes obstáculo
        
        # Crear y colocar el robot
        self.robot = RobotAgent(1, self, start, goal)
        self.schedule.add(self.robot)
        self.grid.place_agent(self.robot, start)
        
        # Añadir recolector de datos para estadísticas
        self.datacollector = DataCollector(
            model_reporters={"Pasos": lambda m: m.robot.steps_taken}
        )
    
    def has_obstacle(self, pos):
        """Comprueba si hay un obstáculo en la posición dada"""
        cell_contents = self.grid.get_cell_list_contents(pos)
        for agent in cell_contents:
            if isinstance(agent, ObstacleAgent):
                return True
        return False
    
    def add_obstacle(self, pos):
        """Añade un obstáculo en la posición especificada"""
        if pos != self.start and pos != self.goal and not self.has_obstacle(pos):
            # Crear un nuevo agente obstáculo
            obstacle_id = len(self.obstacles) + 100  # IDs únicos para obstáculos
            obstacle = ObstacleAgent(obstacle_id, self)
            self.obstacles.append(obstacle)
            self.schedule.add(obstacle)
            self.grid.place_agent(obstacle, pos)
            
            # Recalcular la ruta del robot si ya tiene una ruta
            if hasattr(self.robot, 'path') and self.robot.path:
                self.robot.path = self.robot.astar(self.robot.pos, self.robot.goal)
    
    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()