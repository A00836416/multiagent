# matplotlib_visualization.py
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import io
import base64

def generate_plot():
    # Crear figura y ejes
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Configurar límites del gráfico
    ax.set_xlim(-7, 7)
    ax.set_ylim(-4, 4)
    
    # Dibujar grid
    ax.grid(True)
    
    # Dibujar posiciones iniciales de los robots
    initial_positions = [(-1.5, -2), (-2, 0)]  # Datos del InitialPositions.txt
    robot_colors = ['red', 'blue']
    
    for i, (x, y) in enumerate(initial_positions):
        ax.plot(x, y, 'o', markersize=12, color=robot_colors[i], label=f'Robot {i+1} - Inicio')
    
    # Dibujar objetivos
    target_positions = [
        [-0.86717069892473, -0.277318548387096, 0.286122311827957, -1.01683467741935, 0.673487903225808, -1.37778897849462, 1.54506048387097],
        [-0.356552419354838, 0.550235215053764, -0.497412634408602, 1.52745295698925, 0.629469086021506, -1.36898521505376, -0.999227150537633]
    ]
    
    # Separar por robot para la visualización
    for i, targets in enumerate(target_positions):
        xs = targets
        ys = target_positions[1] if i == 0 else target_positions[0]  # Intercambiar coordenadas y
        
        for j, (x, y) in enumerate(zip(xs[:len(ys)], ys[:len(xs)])):
            marker = 'x' if j > 0 else '*'
            ax.plot(x, y, marker, markersize=10, color=robot_colors[i])
            if j == 0:  # Solo etiquetar el primer objetivo
                ax.annotate(f'G{i+1}', (x, y), xytext=(5, 5), textcoords='offset points')
    
    # Dibujar obstáculos
    obstacle_data = [
        # Obstacle_1.txt
        [[-0.218391135768374, -0.746320221915368, -0.441520221915368, 0.0864088642316258],
         [0.225864543073497, -0.0789354569265029, -0.606864543073497, -0.302064543073497]],
        # Obstacle_2.txt
        [[0.441520221915368, -0.0864088642316257, 0.218391135768374, 0.746320221915368],
         [0.606864543073497, 0.302064543073497, -0.225864543073497, 0.0789354569265031]],
        # Obstacle_3.txt
        [[-0.6952, -1.3048, -1.3048, -0.6952],
         [1.3048, 1.3048, 0.6952, 0.6952]],
        # Obstacle_4.txt
        [[1.3048, 0.6952, 0.6952, 1.3048],
         [-0.6952, -0.6952, -1.3048, -1.3048]],
        # Obstacle_5.txt
        [[0.873302236800156, 0.28447385309434, 0.126697763199844, 0.71552614690566],
         [1.43552614690566, 1.59330223680016, 1.00447385309434, 0.846697763199844]],
        # Obstacle_6.txt
        [[-0.568947706188681, -1, -1.43105229381132, -1],
         [-1, -0.568947706188681, -1, -1.43105229381132]]
    ]
    
    for i, obs in enumerate(obstacle_data):
        xs, ys = obs
        vertices = list(zip(xs, ys))
        polygon = Polygon(vertices, closed=True, fill=True, color='gray', alpha=0.7)
        ax.add_patch(polygon)
    
    # Añadir leyenda y etiquetas
    ax.legend()
    ax.set_title('Simulación de Robots y Obstáculos')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    
    # Convertir la figura a una imagen base64 para mostrar en HTML
    buf = io.BytesIO()
    plt.savefig(buf, format='png')
    buf.seek(0)
    img_data = base64.b64encode(buf.getvalue()).decode('utf-8')
    plt.close(fig)
    
    return img_data