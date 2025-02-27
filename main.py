from mesa import Agent, Model
from mesa.space import MultiGrid

class WallEAgent(Agent):
    def __init__(self, unique_id, model, start, goal):
        self.unique_id = unique_id
        self.model = model
        self.start = start
        self.goal = goal
        self.pos = None
        self.path = self.calculate_path(start, goal)
        print("Ruta calculada:", self.path)
    
    def calculate_path(self, start, goal):
        # Wall-e solo sabe moverse a la derecha
        # Genera una ruta desde start hasta goal moviéndose solo en horizontal
        path = [start]
        x, y = start
        
        # Si la meta está a la izquierda, no puede llegar
        if goal[0] < x:
            print("¡Wall-e no puede llegar al objetivo! Solo sabe moverse a la derecha.")
            return [start]
        
        # Moverse a la derecha hasta alcanzar la coordenada x de la meta
        while x < goal[0]:
            x += 1
            path.append((x, y))
        
        return path

    def step(self):
        if len(self.path) > 1:
            self.path.pop(0)
            next_pos = self.path[0]
            self.model.grid.move_agent(self, next_pos)
            print(f"Wall-e se movió a {next_pos}")
            # Convertir a formato 1-indexado para la salida
            print(f"Posición actual: ({next_pos[0]+1}, {next_pos[1]+1})")
        else:
            print("¡Wall-e ha alcanzado el objetivo!")

class WallEModel(Model):
    def __init__(self, width, height, start, goal):
        super().__init__()
        self.grid = MultiGrid(width, height, torus=False)
        self.schedule = []
        self.start = start
        self.goal = goal
        self.running = True

        # Crear y colocar el agente Wall-e
        self.walle = WallEAgent(1, self, start, goal)
        self.schedule.append(self.walle)
        self.grid.place_agent(self.walle, start)

    def step(self):
        # Implementamos sistema de pasos simple
        for agent in self.schedule:
            agent.step()
        # Detener la simulación cuando Wall-e llega al objetivo
        if len(self.walle.path) <= 1:
            self.running = False

def run_simulation():
    # Nota: Mesa usa coordenadas basadas en 0, así que restamos 1 a las coordenadas
    width = 5
    height = 5
    start = (0, 2) # Equivale a 1, 3
    goal = (4, 2) # Equivale a 5, 3
    
    model = WallEModel(width, height, start, goal)
    step_num = 1
    
    print(f"Iniciando simulación: Wall-e en posición ({start[0]+1}, {start[1]+1}), meta en ({goal[0]+1}, {goal[1]+1})")
    while model.running:
        print(f"\nPaso {step_num}")
        model.step()
        step_num += 1

if __name__ == "__main__":
    run_simulation()