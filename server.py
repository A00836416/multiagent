from flask import Flask, render_template, jsonify, request
from pathfinding_model import RobotAgent, ObstacleAgent, PathFindingModel
import json

app = Flask(__name__)

# Modelo global para mantener el estado
model = None
obstacles = []

@app.route('/')
def index():
    """Ruta principal que muestra la interfaz"""
    return render_template('index.html')

@app.route('/init', methods=['POST'])
def initialize():
    """Inicializa o reinicia el modelo con parámetros dados"""
    global model, obstacles
    
    data = request.json
    width = int(data.get('width', 10))
    height = int(data.get('height', 10))
    start_x = int(data.get('start_x', 1))
    start_y = int(data.get('start_y', 1))
    goal_x = int(data.get('goal_x', 8))
    goal_y = int(data.get('goal_y', 8))
    
    # Inicializar el modelo
    model = PathFindingModel(width, height, (start_x, start_y), (goal_x, goal_y))
    obstacles = []
    
    return jsonify({
        'success': True,
        'grid_size': {'width': width, 'height': height},
        'start': {'x': start_x, 'y': start_y},
        'goal': {'x': goal_x, 'y': goal_y},
        'robot_position': {'x': model.robot.pos[0], 'y': model.robot.pos[1]},
        'path': [{'x': pos[0], 'y': pos[1]} for pos in model.robot.path]
    })

@app.route('/step', methods=['POST'])
def step():
    """Avanza un paso en la simulación"""
    global model
    
    if model is None:
        return jsonify({'error': 'Modelo no inicializado'}), 400
    
    # Ejecutar un paso
    model.step()
    
    # Si el robot ha llegado a la meta
    reached_goal = model.robot.pos == model.robot.goal
    
    return jsonify({
        'success': True,
        'robot_position': {'x': model.robot.pos[0], 'y': model.robot.pos[1]},
        'reached_goal': reached_goal,
        'steps_left': len(model.robot.path) - 1 if model.robot.path else 0
    })

@app.route('/add_obstacle', methods=['POST'])
def add_obstacle():
    """Añade un obstáculo en la posición especificada"""
    global model, obstacles
    
    if model is None:
        return jsonify({'error': 'Modelo no inicializado'}), 400
    
    data = request.json
    x = int(data.get('x', 0))
    y = int(data.get('y', 0))
    
    # Asegurarse de que no se añada un obstáculo en la posición del robot o la meta
    if (x, y) == model.robot.pos or (x, y) == model.robot.goal:
        return jsonify({'error': 'No se puede añadir obstáculo en la posición del robot o la meta'}), 400
    
    # Añadir obstáculo
    model.add_obstacle((x, y))
    obstacles.append({'x': x, 'y': y})
    
    # Recalcular la ruta si el robot tiene una ruta
    if hasattr(model.robot, 'path') and model.robot.path:
        model.robot.path = model.robot.astar(model.robot.pos, model.robot.goal)
    
    # Si no se pudo encontrar una ruta, devolver error
    if not model.robot.path:
        return jsonify({
            'success': False,
            'error': 'No se pudo encontrar una ruta al añadir el obstáculo'
        }), 400
    
    return jsonify({
        'success': True,
        'obstacles': obstacles,
        'path': [{'x': pos[0], 'y': pos[1]} for pos in model.robot.path]
    })

@app.route('/get_state', methods=['GET'])
def get_state():
    """Devuelve el estado actual del modelo"""
    global model, obstacles
    
    if model is None:
        return jsonify({'error': 'Modelo no inicializado'}), 400
    
    return jsonify({
        'grid_size': {'width': model.grid.width, 'height': model.grid.height},
        'start': {'x': model.start[0], 'y': model.start[1]},
        'goal': {'x': model.goal[0], 'y': model.goal[1]},
        'robot_position': {'x': model.robot.pos[0], 'y': model.robot.pos[1]},
        'obstacles': obstacles,
        'path': [{'x': pos[0], 'y': pos[1]} for pos in model.robot.path],
        'reached_goal': model.robot.pos == model.robot.goal,
        'steps_taken': model.robot.steps_taken
    })

if __name__ == '__main__':
    # Crear el directorio templates si no existe
    import os
    if not os.path.exists('templates'):
        os.makedirs('templates')
    
    # Escribir el archivo de plantilla HTML
    with open('templates/index.html', 'w') as f:
        f.write("""<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Pathfinding Simulation</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 1000px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            text-align: center;
            color: #333;
        }
        .controls {
            display: flex;
            gap: 20px;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }
        .control-group {
            flex: 1;
            min-width: 200px;
        }
        .control-item {
            margin-bottom: 10px;
        }
        label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
        }
        input[type="number"] {
            width: 80px;
            padding: 8px;
            border: 1px solid #ddd;
            border-radius: 4px;
        }
        .buttons {
            display: flex;
            gap: 10px;
            margin-bottom: 20px;
        }
        button {
            padding: 10px 15px;
            border: none;
            border-radius: 4px;
            background-color: #4CAF50;
            color: white;
            cursor: pointer;
            font-size: 14px;
            font-weight: bold;
        }
        button:hover {
            background-color: #45a049;
        }
        button:disabled {
            background-color: #cccccc;
            cursor: not-allowed;
        }
        #resetBtn {
            background-color: #f44336;
        }
        #resetBtn:hover {
            background-color: #d32f2f;
        }
        #stepBtn {
            background-color: #2196F3;
        }
        #stepBtn:hover {
            background-color: #0b7dda;
        }
        .grid-container {
            width: 100%;
            aspect-ratio: 1;
            margin: 0 auto;
            position: relative;
            background-color: #f9f9f9;
            border: 1px solid #ddd;
            overflow: hidden;
        }
        #grid {
            width: 100%;
            height: 100%;
            display: grid;
            position: relative;
        }
        .cell {
            border: 1px solid #ddd;
            background-color: white;
            position: relative;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            transition: background-color 0.3s;
        }
        .cell:hover {
            background-color: #f0f0f0;
        }
        .robot {
            position: absolute;
            border-radius: 50%;
            background-color: red;
            color: white;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 24px;
            z-index: 10;
            transition: all 0.5s ease;
        }
        .start {
            background-color: #2196F3 !important;
            color: white;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .goal {
            background-color: #4CAF50 !important;
            color: white;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .obstacle {
            background-color: #333 !important;
        }
        .path {
            background-color: #FFC107 !important;
            opacity: 0.5;
        }
        .status {
            margin-top: 20px;
            padding: 10px;
            background-color: #e9e9e9;
            border-radius: 4px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Solución de Ruta</h1>
        
        <div class="controls">
            <div class="control-group">
                <div class="control-item">
                    <label for="gridWidth">Ancho del Grid:</label>
                    <input type="number" id="gridWidth" min="5" max="20" value="10">
                </div>
                <div class="control-item">
                    <label for="gridHeight">Alto del Grid:</label>
                    <input type="number" id="gridHeight" min="5" max="20" value="10">
                </div>
            </div>
            
            <div class="control-group">
                <div class="control-item">
                    <label for="startX">Inicio X:</label>
                    <input type="number" id="startX" min="0" max="9" value="1">
                </div>
                <div class="control-item">
                    <label for="startY">Inicio Y:</label>
                    <input type="number" id="startY" min="0" max="9" value="1">
                </div>
            </div>
            
            <div class="control-group">
                <div class="control-item">
                    <label for="goalX">Meta X:</label>
                    <input type="number" id="goalX" min="0" max="9" value="8">
                </div>
                <div class="control-item">
                    <label for="goalY">Meta Y:</label>
                    <input type="number" id="goalY" min="0" max="9" value="8">
                </div>
            </div>
        </div>
        
        <div class="buttons">
            <button id="initBtn">Inicializar</button>
            <button id="startBtn" disabled>Iniciar</button>
            <button id="stepBtn" disabled>Paso</button>
            <button id="resetBtn" disabled>Reiniciar</button>
        </div>
        
        <div class="grid-container">
            <div id="grid"></div>
            <div id="robot" class="robot">🤖</div>
        </div>
        
        <div class="status" id="status">Estado: No inicializado</div>
    </div>

    <script>
        // Variables globales
        let gridWidth = 10;
        let gridHeight = 10;
        let startPos = { x: 1, y: 1 };
        let goalPos = { x: 8, y: 8 };
        let robotPos = { x: 1, y: 1 };
        let obstacles = [];
        let path = [];
        let isRunning = false;
        let animationInterval = null;
        
        // Elementos DOM
        const gridElement = document.getElementById('grid');
        const robotElement = document.getElementById('robot');
        const statusElement = document.getElementById('status');
        const initButton = document.getElementById('initBtn');
        const startButton = document.getElementById('startBtn');
        const stepButton = document.getElementById('stepBtn');
        const resetButton = document.getElementById('resetBtn');
        
        // Inicializar el grid
        function initializeGrid() {
            // Obtener valores de los inputs
            gridWidth = parseInt(document.getElementById('gridWidth').value);
            gridHeight = parseInt(document.getElementById('gridHeight').value);
            startPos.x = parseInt(document.getElementById('startX').value);
            startPos.y = parseInt(document.getElementById('startY').value);
            goalPos.x = parseInt(document.getElementById('goalX').value);
            goalPos.y = parseInt(document.getElementById('goalY').value);
            
            // Validar que start y goal estén dentro del grid
            if (startPos.x >= gridWidth || startPos.y >= gridHeight || 
                goalPos.x >= gridWidth || goalPos.y >= gridHeight ||
                startPos.x < 0 || startPos.y < 0 || 
                goalPos.x < 0 || goalPos.y < 0) {
                alert('Las posiciones de inicio y meta deben estar dentro del grid');
                return;
            }
            
            // Limpiar el grid actual
            gridElement.innerHTML = '';
            obstacles = [];
            
            // Configurar el estilo del grid
            gridElement.style.gridTemplateColumns = `repeat(${gridWidth}, 1fr)`;
            gridElement.style.gridTemplateRows = `repeat(${gridHeight}, 1fr)`;
            
            // Crear las celdas
            for (let y = 0; y < gridHeight; y++) {
                for (let x = 0; x < gridWidth; x++) {
                    const cell = document.createElement('div');
                    cell.classList.add('cell');
                    cell.dataset.x = x;
                    cell.dataset.y = y;
                    
                    // Marcar celdas de inicio y meta
                    if (x === startPos.x && y === startPos.y) {
                        cell.classList.add('start');
                        cell.textContent = 'S';
                    } else if (x === goalPos.x && y === goalPos.y) {
                        cell.classList.add('goal');
                        cell.textContent = 'G';
                    }
                    
                    // Añadir evento de clic para agregar obstáculos
                    cell.addEventListener('click', (e) => {
                        const cellX = parseInt(e.target.dataset.x);
                        const cellY = parseInt(e.target.dataset.y);
                        
                        // No permitir obstáculos en posiciones de inicio, meta o robot
                        if ((cellX === startPos.x && cellY === startPos.y) ||
                            (cellX === goalPos.x && cellY === goalPos.y) ||
                            (cellX === robotPos.x && cellY === robotPos.y)) {
                            return;
                        }
                        
                        // Verificar si ya hay un obstáculo
                        const isObstacle = obstacles.some(obs => obs.x === cellX && obs.y === cellY);
                        
                        if (!isObstacle) {
                            // Añadir obstáculo al backend
                            fetch('/add_obstacle', {
                                method: 'POST',
                                headers: {
                                    'Content-Type': 'application/json',
                                },
                                body: JSON.stringify({ x: cellX, y: cellY })
                            })
                            .then(response => response.json())
                            .then(data => {
                                if (data.success) {
                                    // Actualizar UI
                                    e.target.classList.add('obstacle');
                                    obstacles = data.obstacles;
                                    path = data.path;
                                    updatePath();
                                } else {
                                    alert(data.error || 'No se pudo añadir el obstáculo');
                                }
                            })
                            .catch(error => {
                                console.error('Error:', error);
                                alert('Error al comunicarse con el servidor');
                            });
                        }
                    });
                    
                    gridElement.appendChild(cell);
                }
            }
            
            // Inicializar el modelo en el backend
            fetch('/init', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    width: gridWidth,
                    height: gridHeight,
                    start_x: startPos.x,
                    start_y: startPos.y,
                    goal_x: goalPos.x,
                    goal_y: goalPos.y
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    robotPos = data.robot_position;
                    path = data.path;
                    updateRobotPosition();
                    updatePath();
                    updateStatus('Modelo inicializado. Listo para iniciar la simulación.');
                    startButton.disabled = false;
                    stepButton.disabled = false;
                    resetButton.disabled = false;
                }
            })
            .catch(error => {
                console.error('Error:', error);
                updateStatus('Error al inicializar el modelo.');
            });
        }
        
        // Actualizar la posición del robot en el UI
        function updateRobotPosition() {
            const cellWidth = gridElement.offsetWidth / gridWidth;
            const cellHeight = gridElement.offsetHeight / gridHeight;
            
            robotElement.style.width = `${cellWidth * 0.8}px`;
            robotElement.style.height = `${cellHeight * 0.8}px`;
            robotElement.style.left = `${robotPos.x * cellWidth + (cellWidth * 0.1)}px`;
            robotElement.style.top = `${robotPos.y * cellHeight + (cellHeight * 0.1)}px`;
        }
        
        // Actualizar camino en el UI
        function updatePath() {
            // Limpiar camino anterior
            document.querySelectorAll('.cell.path').forEach(cell => {
                cell.classList.remove('path');
            });
            
            // Marcar las celdas del nuevo camino
            path.forEach(pos => {
                // No marcar inicio, meta o posición actual del robot
                if ((pos.x === startPos.x && pos.y === startPos.y) ||
                    (pos.x === goalPos.x && pos.y === goalPos.y) ||
                    (pos.x === robotPos.x && pos.y === robotPos.y)) {
                    return;
                }
                
                const cellIndex = pos.y * gridWidth + pos.x;
                const cell = gridElement.children[cellIndex];
                if (cell) {
                    cell.classList.add('path');
                }
            });
        }
        
        // Actualizar texto de estado
        function updateStatus(message) {
            statusElement.textContent = `Estado: ${message}`;
        }
        
        // Función para ejecutar un paso
        function step() {
            fetch('/step', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    robotPos = data.robot_position;
                    updateRobotPosition();
                    
                    if (data.reached_goal) {
                        updateStatus('¡El robot ha alcanzado la meta!');
                        stopSimulation();
                    } else {
                        updateStatus(`Robot en (${robotPos.x}, ${robotPos.y}). Pasos restantes: ${data.steps_left}`);
                    }
                }
            })
            .catch(error => {
                console.error('Error:', error);
                updateStatus('Error al ejecutar un paso.');
                stopSimulation();
            });
        }
        
        // Iniciar la simulación automática
        function startSimulation() {
            if (!isRunning) {
                isRunning = true;
                startButton.textContent = 'Detener';
                stepButton.disabled = true;
                
                // Ejecutar pasos automáticamente cada segundo
                animationInterval = setInterval(() => {
                    fetch('/get_state')
                    .then(response => response.json())
                    .then(data => {
                        if (data.reached_goal) {
                            updateStatus('¡El robot ha alcanzado la meta!');
                            stopSimulation();
                        } else {
                            step();
                        }
                    })
                    .catch(error => {
                        console.error('Error:', error);
                        stopSimulation();
                    });
                }, 1000);
            } else {
                stopSimulation();
            }
        }
        
        // Detener la simulación
        function stopSimulation() {
            isRunning = false;
            startButton.textContent = 'Iniciar';
            stepButton.disabled = false;
            
            if (animationInterval) {
                clearInterval(animationInterval);
                animationInterval = null;
            }
        }
        
        // Reiniciar la simulación
        function resetSimulation() {
            stopSimulation();
            initializeGrid();
        }
        
        // Event listeners para los botones
        initButton.addEventListener('click', initializeGrid);
        startButton.addEventListener('click', startSimulation);
        stepButton.addEventListener('click', step);
        resetButton.addEventListener('click', resetSimulation);
        
        // Actualizar tamaño del robot cuando cambia el tamaño de la ventana
        window.addEventListener('resize', () => {
            if (robotPos) {
                updateRobotPosition();
            }
        });
        
        // Inicializar al cargar la página
        window.addEventListener('load', () => {
            // Inicializar automáticamente
            initializeGrid();
        });
    </script>
</body>
</html>""")
    
    print("Servidor Flask iniciado en http://127.0.0.1:5000")
    app.run(debug=True)