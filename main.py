from mesa import Agent, Model
from mesa.space import MultiGrid
from mesa.time import BaseScheduler

class RobotAgent(Agent):
    def __init__(self, unique_id, model, start, goal):
        # Inicializamos los atributos esenciales sin llamar a Agent.__init__
        self.unique_id = unique_id
        self.model = model
        self.start = start
        self.goal = goal
        self.pos = None 
        self.path = self.astar(start, goal)
        if not self.path:
            print("No se encontró camino del inicio al objetivo.")
        else:
            print("Ruta calculada:", self.path)
    
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
            print("El robot se movió a", next_pos)
        else:
            print("El robot ha alcanzado el objetivo!")

class PathFindingModel(Model):
    def __init__(self, width, height, start, goal):
        super().__init__()
        self.grid = MultiGrid(width, height, torus=False)
        self.schedule = BaseScheduler(self)
        self.start = start
        self.goal = goal

        self.robot = RobotAgent(1, self, start, goal)
        self.schedule.add(self.robot)
        # Aquí se coloca al agente en la grilla; grid.place_agent asignará self.pos
        self.grid.place_agent(self.robot, start)

    def step(self):
        self.schedule.step()

def get_int_input(prompt):
    while True:
        try:
            return int(input(prompt))
        except ValueError:
            print("Por favor, ingrese un número entero válido.")

def get_coordinate_input(prompt, max_value):
    while True:
        value = get_int_input(prompt)
        if 0 <= value < max_value:
            return value
        else:
            print(f"Valor inválido. Debe estar entre 0 y {max_value - 1}.")

def main():
    # Solicitar tamaño del grid
    width = get_int_input("Ingrese el ancho del grid: ")
    height = get_int_input("Ingrese el alto del grid: ")

    print(f"Las coordenadas deben estar en el rango: x: [0, {width - 1}], y: [0, {height - 1}]")

    # Solicitar coordenadas de inicio y meta con validación
    start_x = get_coordinate_input("Ingrese la posición de inicio x: ", width)
    start_y = get_coordinate_input("Ingrese la posición de inicio y: ", height)
    goal_x = get_coordinate_input("Ingrese la posición meta en x: ", width)
    goal_y = get_coordinate_input("Ingrese la posición meta en y: ", height)

    # Inicializar el modelo
    model = PathFindingModel(width, height, (start_x, start_y), (goal_x, goal_y))

    step_num = 1
    # Ejecutar pasos hasta que el robot alcance el objetivo
    while len(model.robot.path) > 1:
        print(f"Paso {step_num}")
        model.step()
        step_num += 1

    # Paso final (cuando el robot ya se encuentra en el objetivo)
    if model.robot.path:
        model.step()

if __name__ == '__main__':
    main()