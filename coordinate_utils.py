import numpy as np
from shapely.geometry import Polygon, Point

class CoordinateConverter:
    def __init__(self, grid_width, grid_height, scale=1.0):
        """
        Inicializa el conversor de coordenadas.
        
        Args:
            grid_width: Ancho del grid en celdas
            grid_height: Alto del grid en celdas
            scale: Factor de escala para convertir unidades del mundo a celdas del grid
        """
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.scale = scale
        self.origin_x = grid_width // 2
        self.origin_y = grid_height // 2
    
    def world_to_grid(self, world_x, world_y):
        """
        Convierte coordenadas del mundo (centradas en origen) a coordenadas del grid.
        
        Args:
            world_x: Coordenada x del mundo (puede ser decimal)
            world_y: Coordenada y del mundo (puede ser decimal)
            
        Returns:
            Tupla (grid_x, grid_y) con coordenadas en el grid
        """
        # Escalar las coordenadas del mundo
        scaled_x = world_x * self.scale
        scaled_y = world_y * self.scale
        
        # Convertir al sistema de coordenadas del grid
        grid_x = int(round(self.origin_x + scaled_x))
        grid_y = int(round(self.origin_y - scaled_y))  # Invertir Y porque en el grid Y aumenta hacia abajo
        
        # Asegurar que las coordenadas estén dentro del grid
        grid_x = max(0, min(grid_x, self.grid_width - 1))
        grid_y = max(0, min(grid_y, self.grid_height - 1))
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """
        Convierte coordenadas del grid a coordenadas del mundo.
        
        Args:
            grid_x: Coordenada x del grid (entero)
            grid_y: Coordenada y del grid (entero)
            
        Returns:
            Tupla (world_x, world_y) con coordenadas en el mundo
        """
        # Convertir al sistema de coordenadas centrado
        world_x = (grid_x - self.origin_x) / self.scale
        world_y = (self.origin_y - grid_y) / self.scale  # Invertir Y porque en el grid Y aumenta hacia abajo
        
        return world_x, world_y
    
    def load_robot_positions(self, filename):
        """
        Carga posiciones iniciales de robots desde un archivo.
        
        Args:
            filename: Ruta al archivo con formato x1,x2\\ny1,y2
            
        Returns:
            Lista de posiciones de robots [(grid_x1, grid_y1), (grid_x2, grid_y2)]
        """
        with open(filename, 'r') as file:
            lines = file.readlines()
            
        if len(lines) < 2:
            print(f"Error: formato incorrecto en {filename}. Se requieren al menos 2 líneas.")
            return []
            
        x_coords = [float(x) for x in lines[0].strip().split(',')]
        y_coords = [float(y) for y in lines[1].strip().split(',')]
        
        positions = []
        for x, y in zip(x_coords, y_coords):
            grid_x, grid_y = self.world_to_grid(x, y)
            positions.append((grid_x, grid_y))
            
        return positions
    
    def load_obstacles(self, filename):
        """
        Carga las coordenadas de un obstáculo.
        
        Args:
            filename: Ruta al archivo con formato x1,x2,x3,x4\\ny1,y2,y3,y4
            
        Returns:
            Lista de posiciones de las esquinas [(grid_x1, grid_y1), ..., (grid_x4, grid_y4)]
        """
        with open(filename, 'r') as file:
            lines = file.readlines()
            
        x_coords = [float(x) for x in lines[0].strip().split(',')]
        y_coords = [float(y) for y in lines[1].strip().split(',')]
        
        corners = []
        for x, y in zip(x_coords, y_coords):
            grid_x, grid_y = self.world_to_grid(x, y)
            corners.append((grid_x, grid_y))
            
        return corners
    
    def load_target_positions(self, filename):
        """
        Carga las posiciones objetivo para los robots.
        
        Args:
            filename: Ruta al archivo con formato x1,x2,...\\ny1,y2,...
            
        Returns:
            Lista de posiciones objetivo [(grid_x1, grid_y1), (grid_x2, grid_y2), ...]
        """
        with open(filename, 'r') as file:
            lines = file.readlines()
            
        if len(lines) < 2:
            print(f"Error: formato incorrecto en {filename}. Se requieren al menos 2 líneas.")
            return []
            
        x_coords = [float(x) for x in lines[0].strip().split(',')]
        y_coords = [float(y) for y in lines[1].strip().split(',')]
        
        # Verificar que hay la misma cantidad de coordenadas x e y
        if len(x_coords) != len(y_coords):
            print(f"Error: formato incorrecto en {filename}. Diferentes cantidades de coordenadas X e Y.")
            return []
        
        positions = []
        for x, y in zip(x_coords, y_coords):
            grid_x, grid_y = self.world_to_grid(x, y)
            positions.append((grid_x, grid_y))
        
        return positions
    
    def generate_obstacle_cells(self, corners):
        """
        Genera todas las celdas del grid contenidas dentro de un polígono definido por sus esquinas.
        
        Args:
            corners: Lista de coordenadas (grid_x, grid_y) de las esquinas del polígono
            
        Returns:
            Lista de coordenadas (grid_x, grid_y) de todas las celdas dentro del polígono
        """
        # Crear un polígono con las esquinas
        try:
            # Asegurarse que el polígono esté cerrado (último punto = primer punto)
            if corners[0] != corners[-1] and len(corners) >= 3:
                corners.append(corners[0])
                
            polygon = Polygon(corners)
            
            # Encontrar los límites del polígono
            min_x = min(corner[0] for corner in corners)
            max_x = max(corner[0] for corner in corners)
            min_y = min(corner[1] for corner in corners)
            max_y = max(corner[1] for corner in corners)
            
            # Generar todas las celdas que están dentro del polígono
            cells = []
            for x in range(min_x, max_x + 1):
                for y in range(min_y, max_y + 1):
                    if polygon.contains(Point(x, y)):
                        cells.append((x, y))
            
            return cells
        except Exception as e:
            print(f"Error al generar celdas para el obstáculo: {e}")
            # En caso de error, simplemente devolver las esquinas
            return corners