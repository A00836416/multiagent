from flask import Flask, render_template, jsonify, request
from pathfinding_model import RobotAgent, ObstacleAgent, PathFindingModel
import json

app = Flask(__name__)

# Modelo global para mantener el estado
model = None
obstacles = []
robots_config = []

@app.route('/')
def index():
    """Ruta principal que muestra la interfaz"""
    return render_template('index.html')

@app.route('/init', methods=['POST'])
def initialize():
    """Inicializa o reinicia el modelo con parámetros dados"""
    global model, obstacles, robots_config
    
    data = request.json
    width = int(data.get('width', 10))
    height = int(data.get('height', 10))
    robots_config = data.get('robots', [])
    
    # Asegurar que todos los robots tengan configuración de inicio y meta
    for i, robot in enumerate(robots_config):
        if 'start' not in robot or 'goal' not in robot:
            return jsonify({'error': f'Configuración incompleta para el robot {i+1}'}), 400
    
    # Inicializar el modelo con múltiples robots
    model = PathFindingModel(width, height, robots_config)
    obstacles = []
    
    # Preparar la respuesta con información de todos los robots
    robots_info = []
    for i, robot in enumerate(model.robots):
        robots_info.append({
            'id': robot.unique_id,
            'start': {'x': robot.start[0], 'y': robot.start[1]},
            'goal': {'x': robot.goal[0], 'y': robot.goal[1]},
            'position': {'x': robot.pos[0], 'y': robot.pos[1]},
            'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path],
            'color': robot.color
        })
    
    return jsonify({
        'success': True,
        'grid_size': {'width': width, 'height': height},
        'robots': robots_info
    })

@app.route('/step', methods=['POST'])
def step():
    """Avanza un paso en la simulación"""
    global model
    
    if model is None:
        return jsonify({'error': 'Modelo no inicializado'}), 400
    
    # Ejecutar un paso
    model.step()
    
    # Obtener información actualizada de todos los robots
    robots_info = []
    all_reached_goal = True
    
    for robot in model.robots:
        reached_goal = robot.reached_goal
        all_reached_goal = all_reached_goal and reached_goal
        
        robots_info.append({
            'id': robot.unique_id,
            'position': {'x': robot.pos[0], 'y': robot.pos[1]},
            'reached_goal': reached_goal,
            'steps_left': len(robot.path) - 1 if robot.path else 0,
            'steps_taken': robot.steps_taken
        })
    
    return jsonify({
        'success': True,
        'robots': robots_info,
        'all_reached_goal': all_reached_goal
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
    
    # Añadir obstáculo (la función add_obstacle ya verifica si la posición es válida)
    success = model.add_obstacle((x, y))
    
    if success:
        obstacles.append({'x': x, 'y': y})
        
        # Obtener información actualizada de las rutas de todos los robots
        robots_paths = []
        for robot in model.robots:
            if not robot.path:
                return jsonify({
                    'success': False,
                    'error': f'No se pudo encontrar una ruta para el robot {robot.unique_id} al añadir el obstáculo'
                }), 400
            
            robots_paths.append({
                'id': robot.unique_id,
                'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path]
            })
        
        return jsonify({
            'success': True,
            'obstacles': obstacles,
            'robots_paths': robots_paths
        })
    else:
        return jsonify({
            'success': False,
            'error': 'No se puede añadir obstáculo en la posición de un robot o su meta'
        }), 400

@app.route('/add_robot', methods=['POST'])
def add_robot():
    """Añade un nuevo robot a la simulación"""
    global model, robots_config
    
    if model is None:
        return jsonify({'error': 'Modelo no inicializado'}), 400
    
    data = request.json
    start_x = int(data.get('start_x', 0))
    start_y = int(data.get('start_y', 0))
    goal_x = int(data.get('goal_x', 0))
    goal_y = int(data.get('goal_y', 0))
    color = data.get('color', "blue")  # Color por defecto
    
    start = (start_x, start_y)
    goal = (goal_x, goal_y)
    
    # Verificar que las posiciones no estén ocupadas por obstáculos
    if model.has_obstacle(start) or model.has_obstacle(goal):
        return jsonify({
            'success': False,
            'error': 'Las posiciones de inicio o meta están ocupadas por obstáculos'
        }), 400
    
    # Crear nuevo robot
    new_robot_id = len(model.robots) + 1
    new_robot = RobotAgent(new_robot_id, model, start, goal, color)
    
    # Añadir a la lista de robots y al programador
    model.robots.append(new_robot)
    model.schedule.add(new_robot)
    model.grid.place_agent(new_robot, start)
    
    # Actualizar configuración de robots
    robots_config.append({
        'start': start,
        'goal': goal,
        'color': color
    })
    
    if not new_robot.path:
        return jsonify({
            'success': False,
            'error': 'No se pudo encontrar una ruta para el nuevo robot'
        }), 400
    
    return jsonify({
        'success': True,
        'robot': {
            'id': new_robot.unique_id,
            'start': {'x': start[0], 'y': start[1]},
            'goal': {'x': goal[0], 'y': goal[1]},
            'position': {'x': start[0], 'y': start[1]},
            'path': [{'x': pos[0], 'y': pos[1]} for pos in new_robot.path],
            'color': color
        }
    })

@app.route('/get_state', methods=['GET'])
def get_state():
    """Devuelve el estado actual del modelo"""
    global model, obstacles
    
    if model is None:
        return jsonify({'error': 'Modelo no inicializado'}), 400
    
    # Obtener información de todos los robots
    robots_info = []
    for robot in model.robots:
        robots_info.append({
            'id': robot.unique_id,
            'start': {'x': robot.start[0], 'y': robot.start[1]},
            'goal': {'x': robot.goal[0], 'y': robot.goal[1]},
            'position': {'x': robot.pos[0], 'y': robot.pos[1]},
            'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path],
            'reached_goal': robot.reached_goal,
            'steps_taken': robot.steps_taken,
            'color': robot.color
        })
    
    return jsonify({
        'grid_size': {'width': model.grid.width, 'height': model.grid.height},
        'robots': robots_info,
        'obstacles': obstacles,
        'all_reached_goal': model.all_robots_reached_goal()
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
            max-width: 1200px;
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
        input[type="number"], select {
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
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 24px;
            z-index: 10;
            transition: all 0.5s ease;
        }
        .robot-emoji {
            font-size: 24px;
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
        .robot-controls {
            margin-top: 20px;
            border: 1px solid #ddd;
            padding: 15px;
            border-radius: 4px;
        }
        .robot-list {
            margin-top: 10px;
        }
        .robot-item {
            display: flex;
            align-items: center;
            margin-bottom: 5px;
            padding: 5px;
            border: 1px solid #eee;
            border-radius: 4px;
        }
        .robot-color {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            margin-right: 10px;
        }
        .robot-info {
            flex: 1;
        }
        .robot-status {
            margin-left: 10px;
            font-weight: bold;
        }
        .robot-status.completed {
            color: green;
        }
        .robot-status.in-progress {
            color: blue;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Simulación de Búsqueda de Ruta con Múltiples Robots</h1>
        
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
        </div>
        
        <div class="robot-controls">
            <h3>Añadir Robot</h3>
            <div class="controls">
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
                
                <div class="control-group">
                    <div class="control-item">
                        <label for="robotColor">Color:</label>
                        <select id="robotColor">
                            <option value="red">Rojo</option>
                            <option value="blue">Azul</option>
                            <option value="green">Verde</option>
                            <option value="purple">Morado</option>
                            <option value="orange">Naranja</option>
                            <option value="brown">Marrón</option>
                            <option value="pink">Rosa</option>
                            <option value="teal">Turquesa</option>
                        </select>
                    </div>
                    <div class="control-item">
                        <button id="addRobotBtn">Añadir Robot</button>
                    </div>
                </div>
            </div>
            
            <div class="robot-list" id="robotList">
                <!-- Aquí se añadirán los robots -->
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
            <!-- Los robots se añadirán dinámicamente aquí -->
            <div id="robots-container"></div>
        </div>
        
        <div class="status" id="status">Estado: No inicializado</div>
    </div>

    <script>
        // Variables globales
        let gridWidth = 10;
        let gridHeight = 10;
        let robots = [];
        let obstacles = [];
        let isRunning = false;
        let animationInterval = null;
        
        // Elementos DOM
        const gridElement = document.getElementById('grid');
        const robotsContainer = document.getElementById('robots-container');
        const robotList = document.getElementById('robotList');
        const statusElement = document.getElementById('status');
        const initButton = document.getElementById('initBtn');
        const startButton = document.getElementById('startBtn');
        const stepButton = document.getElementById('stepBtn');
        const resetButton = document.getElementById('resetBtn');
        const addRobotButton = document.getElementById('addRobotBtn');
        
        // Función para añadir un robot a la interfaz antes de inicializar
        function addRobotToUI() {
            const startX = parseInt(document.getElementById('startX').value);
            const startY = parseInt(document.getElementById('startY').value);
            const goalX = parseInt(document.getElementById('goalX').value);
            const goalY = parseInt(document.getElementById('goalY').value);
            const color = document.getElementById('robotColor').value;
            
            // Validar que las coordenadas estén dentro del grid
            if (startX >= gridWidth || startY >= gridHeight || 
                goalX >= gridWidth || goalY >= gridHeight ||
                startX < 0 || startY < 0 || 
                goalX < 0 || goalY < 0) {
                alert('Las posiciones de inicio y meta deben estar dentro del grid');
                return;
            }
            
            const robotId = robots.length + 1;
            const robot = {
                id: robotId,
                start: {x: startX, y: startY},
                goal: {x: goalX, y: goalY},
                position: {x: startX, y: startY},
                color: color,
                path: []
            };
            
            robots.push(robot);
            updateRobotList();
        }
        
        // Función para actualizar la lista de robots en la UI
        function updateRobotList() {
            robotList.innerHTML = '';
            
            if (robots.length === 0) {
                robotList.innerHTML = '<p>No hay robots añadidos. Añade al menos un robot para iniciar la simulación.</p>';
                return;
            }
            
            robots.forEach((robot, index) => {
                const robotItem = document.createElement('div');
                robotItem.classList.add('robot-item');
                
                const colorDiv = document.createElement('div');
                colorDiv.classList.add('robot-color');
                colorDiv.style.backgroundColor = robot.color;
                
                const infoDiv = document.createElement('div');
                infoDiv.classList.add('robot-info');
                infoDiv.innerHTML = `Robot ${robot.id}: Inicio (${robot.start.x}, ${robot.start.y}) → Meta (${robot.goal.x}, ${robot.goal.y})`;
                
                const statusDiv = document.createElement('div');
                statusDiv.classList.add('robot-status');
                if (robot.reached_goal) {
                    statusDiv.classList.add('completed');
                    statusDiv.textContent = 'Completado';
                } else {
                    statusDiv.classList.add('in-progress');
                    statusDiv.textContent = 'En progreso';
                }
                
                robotItem.appendChild(colorDiv);
                robotItem.appendChild(infoDiv);
                
                if (robot.steps_taken !== undefined) {
                    robotItem.appendChild(statusDiv);
                }
                
                robotList.appendChild(robotItem);
            });
        }
        
        // Inicializar el grid
        function initializeGrid() {
            // Obtener valores de los inputs
            gridWidth = parseInt(document.getElementById('gridWidth').value);
            gridHeight = parseInt(document.getElementById('gridHeight').value);
            
            // Validar que hay al menos un robot
            if (robots.length === 0) {
                alert('Añade al menos un robot antes de inicializar');
                return;
            }
            
            // Limpiar el grid actual
            gridElement.innerHTML = '';
            robotsContainer.innerHTML = '';
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
                    
                    // Marcar celdas de inicio y meta para cada robot
                    let isStartOrGoal = false;
                    for (const robot of robots) {
                        if (x === robot.start.x && y === robot.start.y) {
                            cell.classList.add('start');
                            cell.textContent = 'S' + robot.id;
                            cell.style.backgroundColor = robot.color;
                            isStartOrGoal = true;
                            break;
                        } else if (x === robot.goal.x && y === robot.goal.y) {
                            cell.classList.add('goal');
                            cell.textContent = 'G' + robot.id;
                            cell.style.backgroundColor = robot.color;
                            isStartOrGoal = true;
                            break;
                        }
                    }
                    
                    // Añadir evento de clic para agregar obstáculos
                    cell.addEventListener('click', (e) => {
                        const cellX = parseInt(e.target.dataset.x);
                        const cellY = parseInt(e.target.dataset.y);
                        
                        // No permitir obstáculos en posiciones de inicio, meta o robots
                        let isRobotPos = false;
                        for (const robot of robots) {
                            if ((cellX === robot.start.x && cellY === robot.start.y) ||
                                (cellX === robot.goal.x && cellY === robot.goal.y) ||
                                (cellX === robot.position.x && cellY === robot.position.y)) {
                                isRobotPos = true;
                                break;
                            }
                        }
                        
                        if (isRobotPos) {
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
                                    
                                    // Actualizar rutas de los robots
                                    for (const robotPath of data.robots_paths) {
                                        const robot = robots.find(r => r.id === robotPath.id);
                                        if (robot) {
                                            robot.path = robotPath.path;
                                        }
                                    }
                                    
                                    updatePaths();
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
            
            // Crear elementos para los robots
            createRobotElements();
            
            // Inicializar el modelo en el backend
            fetch('/init', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    width: gridWidth,
                    height: gridHeight,
                    robots: robots.map(robot => ({
                        start: [robot.start.x, robot.start.y],
                        goal: [robot.goal.x, robot.goal.y],
                        color: robot.color
                    }))
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    // Actualizar información de los robots desde el backend
                    robots = data.robots;
                    
                    // Actualizar la UI
                    updateRobotPositions();
                    updatePaths();
                    updateRobotList();
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
        
        // Crear elementos para los robots
        function createRobotElements() {
            robotsContainer.innerHTML = '';
            
            const cellWidth = gridElement.offsetWidth / gridWidth;
            const cellHeight = gridElement.offsetHeight / gridHeight;
            
            robots.forEach(robot => {
                const robotElement = document.createElement('div');
                robotElement.id = `robot-${robot.id}`;
                robotElement.classList.add('robot');
                robotElement.style.backgroundColor = robot.color;
                robotElement.style.color = getContrastColor(robot.color);
                
                const robotEmoji = document.createElement('span');
                robotEmoji.classList.add('robot-emoji');
                robotEmoji.textContent = '🤖';
                
                robotElement.appendChild(robotEmoji);
                robotsContainer.appendChild(robotElement);
            });
            
            updateRobotPositions();
        }
        
        // Obtener color de contraste para texto legible
        function getContrastColor(hexColor) {
            // Si es un nombre de color, convertirlo a hex
            const tempElem = document.createElement('div');
            tempElem.style.color = hexColor;
            document.body.appendChild(tempElem);
            const rgbColor = window.getComputedStyle(tempElem).color;
            document.body.removeChild(tempElem);
            
            // Extraer componentes RGB
            const rgbMatch = rgbColor.match(/^rgb\s*\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)$/i);
            if (rgbMatch) {
                const r = parseInt(rgbMatch[1]);
                const g = parseInt(rgbMatch[2]);
                const b = parseInt(rgbMatch[3]);
                
                // Calcular brillo según fórmula estándar
                const brightness = (r * 299 + g * 587 + b * 114) / 1000;
                
                return brightness > 125 ? 'black' : 'white';
            }
            
            return 'white';  // Default
        }
        
        // Actualizar las posiciones de todos los robots en el UI
        function updateRobotPositions() {
            const cellWidth = gridElement.offsetWidth / gridWidth;
            const cellHeight = gridElement.offsetHeight / gridHeight;
            
            robots.forEach(robot => {
                const robotElement = document.getElementById(`robot-${robot.id}`);
                if (robotElement) {
                    robotElement.style.width = `${cellWidth * 0.8}px`;
                    robotElement.style.height = `${cellHeight * 0.8}px`;
                    robotElement.style.left = `${robot.position.x * cellWidth + (cellWidth * 0.1)}px`;
                    robotElement.style.top = `${robot.position.y * cellHeight + (cellHeight * 0.1)}px`;
                }
            });
        }
        
        // Actualizar caminos de todos los robots en el UI
        function updatePaths() {
            // Limpiar caminos anteriores
            document.querySelectorAll('.cell.path').forEach(cell => {
                cell.classList.remove('path');
            });
            
            // Marcar las celdas de los nuevos caminos para cada robot
            robots.forEach(robot => {
                if (!robot.path) return;
                
                robot.path.forEach(pos => {
                    // No marcar inicio, meta o posición actual del robot
                    let isStartGoalOrRobotPos = false;
                    
                    for (const r of robots) {
                        if ((pos.x === r.start.x && pos.y === r.start.y) ||
                            (pos.x === r.goal.x && pos.y === r.goal.y) ||
                            (pos.x === r.position.x && pos.y === r.position.y)) {
                            isStartGoalOrRobotPos = true;
                            break;
                        }
                    }
                    
                    if (isStartGoalOrRobotPos) {
                        return;
                    }
                    
                    const cellIndex = pos.y * gridWidth + pos.x;
                    const cell = gridElement.children[cellIndex];
                    if (cell && !cell.classList.contains('obstacle')) {
                        cell.classList.add('path');
                        // Podríamos establecer un color diferente para el camino de cada robot
                        // cell.style.backgroundColor = robot.color;
                        // cell.style.opacity = "0.3";
                    }
                });
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
                    // Actualizar la información de los robots
                    data.robots.forEach(robotUpdate => {
                        const robot = robots.find(r => r.id === robotUpdate.id);
                        if (robot) {
                            robot.position = robotUpdate.position;
                            robot.reached_goal = robotUpdate.reached_goal;
                            robot.steps_taken = robotUpdate.steps_taken;
                        }
                    });
                    
                    // Actualizar la UI
                    updateRobotPositions();
                    updateRobotList();
                    
                    if (data.all_reached_goal) {
                        updateStatus('¡Todos los robots han alcanzado sus metas!');
                        stopSimulation();
                    } else {
                        const robotsInProgress = robots.filter(r => !r.reached_goal).length;
                        updateStatus(`${robotsInProgress} robots aún en movimiento. ${robots.length - robotsInProgress} han llegado a la meta.`);
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
                        if (data.all_reached_goal) {
                            updateStatus('¡Todos los robots han alcanzado sus metas!');
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
        
        // Añadir un robot a la simulación
        function addRobot() {
            if (isRunning) {
                alert('Detén la simulación antes de añadir un robot');
                return;
            }
            
            const startX = parseInt(document.getElementById('startX').value);
            const startY = parseInt(document.getElementById('startY').value);
            const goalX = parseInt(document.getElementById('goalX').value);
            const goalY = parseInt(document.getElementById('goalY').value);
            const color = document.getElementById('robotColor').value;
            
            // Si el modelo ya está inicializado, añadir robot al backend
            if (startButton.disabled === false) {
                fetch('/add_robot', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        start_x: startX,
                        start_y: startY,
                        goal_x: goalX,
                        goal_y: goalY,
                        color: color
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        robots.push(data.robot);
                        
                        // Actualizar la UI
                        initializeGrid();  // Esto redibuja todo el grid
                    } else {
                        alert(data.error || 'No se pudo añadir el robot');
                    }
                })
                .catch(error => {
                    console.error('Error:', error);
                    alert('Error al comunicarse con el servidor');
                });
            } else {
                // Si el modelo no está inicializado, solo añadirlo a la lista local
                addRobotToUI();
            }
        }
        
        // Event listeners para los botones
        initButton.addEventListener('click', initializeGrid);
        startButton.addEventListener('click', startSimulation);
        stepButton.addEventListener('click', step);
        resetButton.addEventListener('click', resetSimulation);
        addRobotButton.addEventListener('click', addRobot);
        
        // Actualizar tamaño de los robots cuando cambia el tamaño de la ventana
        window.addEventListener('resize', () => {
            updateRobotPositions();
        });
        
        // Inicializar al cargar la página
        window.addEventListener('load', () => {
            // Añadir un robot por defecto
            addRobotToUI();
            updateRobotList();
        });
    </script>
</body>
</html>""")
    
    print("Servidor Flask iniciado en http://127.0.0.1:5000")
    app.run(debug=True)