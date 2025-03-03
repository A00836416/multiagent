import os
import time
import tempfile
import datetime
from flask import Flask, render_template, jsonify, request, send_file
from pathfinding_model import RobotAgent, ObstacleAgent, ChargingStation, PathFindingModel
import json
from flask_cors import CORS  # Para permitir conexiones desde Unity

app = Flask(__name__)
CORS(app)  # Habilitar CORS para todas las rutas

# Modelo global para mantener el estado
model = None
obstacles = []
charging_stations = []
robots_config = []

@app.route('/')
def index():
    """Ruta principal que muestra la interfaz"""
    return render_template('index.html')

@app.route('/init', methods=['POST'])
def initialize():
    """Inicializa o reinicia el modelo con parámetros dados"""
    global model, obstacles, charging_stations, robots_config
    
    data = request.json
    width = int(data.get('width', 10))
    height = int(data.get('height', 10))
    robots_config = data.get('robots', [])
    charging_stations_config = data.get('charging_stations', [])
    obstacles_list = data.get('obstacles', [])  
    
    # Asegurar que todos los robots tengan configuración de inicio y meta
    for i, robot in enumerate(robots_config):
        if 'start' not in robot or 'goal' not in robot:
            return jsonify({'error': f'Configuración incompleta para el robot {i+1}'}), 400
    
    # Inicializar el modelo con múltiples robots y estaciones de carga
    model = PathFindingModel(width, height, robots_config, charging_stations_config)
    
    # Añadir obstáculos si se proporcionan
    obstacles = []
    if obstacles_list:
        for obs in obstacles_list:
            x, y = obs.get('x', 0), obs.get('y', 0)
            if model.add_obstacle((x, y)):
                obstacles.append({'x': x, 'y': y})
    
    # Guardar posiciones de las estaciones de carga
    charging_stations = []
    for station in model.charging_stations:
        charging_stations.append({'x': station.pos[0], 'y': station.pos[1]})
    
    # Preparar la respuesta con información de todos los robots
    robots_info = []
    for robot in model.robots:
        robots_info.append({
            'id': robot.unique_id,
            'start': {'x': robot.start[0], 'y': robot.start[1]},
            'goal': {'x': robot.goal[0], 'y': robot.goal[1]},
            'position': {'x': robot.pos[0], 'y': robot.pos[1]},
            'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path],
            'color': robot.color,
            'battery_level': robot.battery_level,
            'max_battery': robot.max_battery,
            'charging': robot.charging,
            'battery_percentage': (robot.battery_level / robot.max_battery) * 100
        })
    
    return jsonify({
        'success': True,
        'grid_size': {'width': width, 'height': height},
        'robots': robots_info,
        'obstacles': obstacles,
        'charging_stations': charging_stations
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
            'steps_taken': robot.steps_taken,
            'battery_level': robot.battery_level,
            'max_battery': robot.max_battery,
            'charging': robot.charging,
            'status': 'charging' if robot.charging else 'goal_reached' if robot.reached_goal else 'moving',
            'battery_percentage': (robot.battery_level / robot.max_battery) * 100,
            'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path]  # Incluir la ruta actualizada
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
    success = model.add_obstacle((x, y))  # Pasar como tupla
    
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
            'error': 'No se puede añadir obstáculo en la posición especificada'
        }), 400

@app.route('/add_charging_station', methods=['POST'])
def add_charging_station():
    """Añade una estación de carga en la posición especificada"""
    global model, charging_stations
    
    if model is None:
        return jsonify({'error': 'Modelo no inicializado'}), 400
    
    data = request.json
    x = int(data.get('x', 0))
    y = int(data.get('y', 0))
    charging_rate = float(data.get('charging_rate', 10))
    
    # Añadir estación de carga
    success = model.add_charging_station((x, y))
    
    if success:
        charging_stations.append({'x': x, 'y': y})
        
        # También actualizar rutas de robots que podrían estar buscando estaciones
        robots_paths = []
        for robot in model.robots:
            if not robot.reached_goal and robot.battery_level < robot.max_battery * 0.3:  # Batería baja
                # El robot puede recalcular su ruta para usar esta nueva estación
                robot.nearest_charging_station = robot.find_nearest_charging_station()
                if robot.nearest_charging_station:
                    robot.path = robot.calculate_path_to_station(robot.nearest_charging_station)
            
            robots_paths.append({
                'id': robot.unique_id,
                'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path]
            })
            
        return jsonify({
            'success': True,
            'charging_stations': charging_stations,
            'robots_paths': robots_paths  # Incluir rutas actualizadas
        })
    else:
        return jsonify({
            'success': False,
            'error': 'No se puede añadir estación de carga en la posición especificada'
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
    
    # Parámetros de batería
    max_battery = float(data.get('max_battery', 100))
    battery_level = float(data.get('battery_level', max_battery))
    battery_drain_rate = float(data.get('battery_drain_rate', 1))
    
    start = (start_x, start_y)  # Usar tupla
    goal = (goal_x, goal_y)  # Usar tupla
    
    # Verificar que las posiciones no estén ocupadas por obstáculos
    if model.has_obstacle(start) or model.has_obstacle(goal):
        return jsonify({
            'success': False,
            'error': 'Las posiciones de inicio o meta están ocupadas por obstáculos'
        }), 400
    
    # Crear nuevo robot
    new_robot_id = len(model.robots) + 1
    new_robot = RobotAgent(
        new_robot_id, model, start, goal, color,
        max_battery=max_battery,
        battery_drain_rate=battery_drain_rate,
        battery_level=battery_level
    )
    
    # Añadir a la lista de robots y al programador
    model.robots.append(new_robot)
    model.schedule.add(new_robot)
    model.grid.place_agent(new_robot, start)
    
    # Actualizar configuración de robots
    robots_config.append({
        'start': start,
        'goal': goal,
        'color': color,
        'max_battery': max_battery,
        'battery_drain_rate': battery_drain_rate,
        'battery_level': battery_level
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
            'color': color,
            'battery_level': battery_level,
            'max_battery': max_battery,
            'charging': False,
            'battery_percentage': (battery_level / max_battery) * 100
        }
    })

@app.route('/get_state', methods=['GET'])
def get_state():
    """Devuelve el estado actual del modelo"""
    global model, obstacles, charging_stations
    
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
            'color': robot.color,
            'battery_level': robot.battery_level,
            'max_battery': robot.max_battery,
            'charging': robot.charging,
            'battery_percentage': (robot.battery_level / robot.max_battery) * 100
        })
    
    return jsonify({
        'grid_size': {'width': model.grid.width, 'height': model.grid.height},
        'robots': robots_info,
        'obstacles': obstacles,
        'charging_stations': charging_stations,
        'all_reached_goal': model.all_robots_reached_goal()
    })

@app.route('/export_path_coordinates', methods=['GET'])
def export_path_coordinates():
    """Exporta las coordenadas de las rutas de robots en un formato específico"""
    global model
    
    if model is None:
        return jsonify({'error': 'Modelo no inicializado'}), 400
    
    output = []
    
    # Procesar cada robot
    for robot in model.robots:
        # Añadir identificador del robot
        
        # Extraer coordenadas x e y por separado
        x_coords = []
        y_coords = []
        
        for pos in robot.path:
            x_coords.append(str(pos[0]))
            y_coords.append(str(pos[1]))
        
        # Agregar coordenadas en el formato requerido
        output.append(",".join(x_coords))
        output.append(",".join(y_coords))
        output.append("")  # Línea en blanco entre robots
    
    # Generar nombre de archivo con timestamp
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"TargetPositions_{timestamp}.txt"
    
    # Crear archivo en directorio temporal
    temp_dir = tempfile.gettempdir()
    file_path = os.path.join(temp_dir, filename)
    
    # Guardar el archivo
    with open(file_path, 'w') as f:
        f.write('\n'.join(output))
    
    # Devolver el archivo para descargar
    return send_file(file_path, as_attachment=True)

if __name__ == '__main__':
    # Crear el directorio templates si no existe
    import os
    if not os.path.exists('templates'):
        os.makedirs('templates')
    
    # Modificar el HTML para agregar funcionalidad al botón de exportar
    with open('templates/index.html', 'w') as f:
        f.write("""<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Simulación de Robots con Batería</title>
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
        .charging-station {
            background-color: #FFD700 !important;
            border: 2px solid #DAA520;
        }
        .charging-station::before {
            content: "⚡";
            font-size: 1.2em;
        }
        .start {
            background-color: #2196F3 !important;
            color: white;
        }
        .goal {
            background-color: #4CAF50 !important;
            color: white;
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
        .battery-indicator {
            width: 30px;
            height: 15px;
            position: absolute;
            border: 1px solid #333;
            top: -20px;
            left: 50%;
            transform: translateX(-50%);
        }
        .battery-level {
            height: 100%;
            background-color: #4CAF50;
        }
        .battery-low {
            background-color: #ff5722;
        }
        .charging-icon {
            position: absolute;
            top: -15px;
            right: -15px;
            font-size: 12px;
        }
        
        #exportCoordsBtn {
            background-color: #9c27b0;  /* Color morado para diferenciarlo */
        }
        #exportCoordsBtn:hover {
            background-color: #7b1fa2;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Simulación de Robots con Batería</h1>
        
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
        
        <div class="controls">
            <div class="control-group">
                <h3>Añadir Robot</h3>
                <div class="control-item">
                    <label for="startX">Inicio X:</label>
                    <input type="number" id="startX" min="0" max="9" value="1">
                </div>
                <div class="control-item">
                    <label for="startY">Inicio Y:</label>
                    <input type="number" id="startY" min="0" max="9" value="1">
                </div>
                <div class="control-item">
                    <label for="goalX">Meta X:</label>
                    <input type="number" id="goalX" min="0" max="9" value="8">
                </div>
                <div class="control-item">
                    <label for="goalY">Meta Y:</label>
                    <input type="number" id="goalY" min="0" max="9" value="8">
                </div>
                <div class="control-item">
                    <label for="robotColor">Color:</label>
                    <select id="robotColor">
                        <option value="red">Rojo</option>
                        <option value="blue">Azul</option>
                        <option value="green">Verde</option>
                        <option value="purple">Morado</option>
                        <option value="orange">Naranja</option>
                    </select>
                </div>
                <div class="control-item">
                    <label for="maxBattery">Capacidad de Batería:</label>
                    <input type="number" id="maxBattery" min="10" max="1000" value="100">
                </div>
                <div class="control-item">
                    <label for="batteryDrainRate">Tasa de Descarga:</label>
                    <input type="number" id="batteryDrainRate" min="0.1" max="10" step="0.1" value="1">
                </div>
                <div class="control-item">
                    <button id="addRobotBtn">Añadir Robot</button>
                </div>
            </div>
            
            <div class="control-group">
                <h3>Añadir Estación de Carga</h3>
                <div class="control-item">
                    <label for="stationX">Posición X:</label>
                    <input type="number" id="stationX" min="0" max="9" value="5">
                </div>
                <div class="control-item">
                    <label for="stationY">Posición Y:</label>
                    <input type="number" id="stationY" min="0" max="9" value="5">
                </div>
                <div class="control-item">
                    <button id="addStationBtn">Añadir Estación</button>
                </div>
            </div>
        </div>
        
        <div class="buttons">
            <button id="initBtn">Inicializar</button>
            <button id="startBtn" disabled>Iniciar</button>
            <button id="stepBtn" disabled>Paso</button>
            <button id="resetBtn" disabled>Reiniciar</button>
            <button id="exportCoordsBtn" disabled>Exportar Coordenadas</button>
        </div>
        
        <div class="grid-container">
            <div id="grid"></div>
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
        let chargingStations = [];
        let isRunning = false;
        let animationInterval = null;
        
        // Elementos DOM
        const gridElement = document.getElementById('grid');
        const robotsContainer = document.getElementById('robots-container');
        const statusElement = document.getElementById('status');
        const initButton = document.getElementById('initBtn');
        const startButton = document.getElementById('startBtn');
        const stepButton = document.getElementById('stepBtn');
        const resetButton = document.getElementById('resetBtn');
        const exportCoordsButton = document.getElementById('exportCoordsBtn');
        const addRobotButton = document.getElementById('addRobotBtn');
        const addStationButton = document.getElementById('addStationBtn');

        
        
        // Función para añadir un robot
        function addRobot() {
            const startX = parseInt(document.getElementById('startX').value);
            const startY = parseInt(document.getElementById('startY').value);
            const goalX = parseInt(document.getElementById('goalX').value);
            const goalY = parseInt(document.getElementById('goalY').value);
            const color = document.getElementById('robotColor').value;
            const maxBattery = parseFloat(document.getElementById('maxBattery').value);
            const batteryDrainRate = parseFloat(document.getElementById('batteryDrainRate').value);
            
            // Validar que las coordenadas estén dentro del grid
            if (startX >= gridWidth || startY >= gridHeight || 
                goalX >= gridWidth || goalY >= gridHeight ||
                startX < 0 || startY < 0 || 
                goalX < 0 || goalY < 0) {
                alert('Las posiciones de inicio y meta deben estar dentro del grid');
                return;
            }
            
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
                        color: color,
                        max_battery: maxBattery,
                        battery_drain_rate: batteryDrainRate,
                        battery_level: maxBattery
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        robots.push(data.robot);
                        updateGrid();
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
                const robotId = robots.length + 1;
                robots.push({
                    id: robotId,
                    start: {x: startX, y: startY},
                    goal: {x: goalX, y: goalY},
                    position: {x: startX, y: startY},
                    color: color,
                    max_battery: maxBattery,
                    battery_drain_rate: batteryDrainRate,
                    battery_level: maxBattery,
                    path: []
                });
            }
        }
        
        // Función para añadir una estación de carga
        function addChargingStation() {
            const x = parseInt(document.getElementById('stationX').value);
            const y = parseInt(document.getElementById('stationY').value);
            
            // Validar que las coordenadas estén dentro del grid
            if (x >= gridWidth || y >= gridHeight || x < 0 || y < 0) {
                alert('La posición debe estar dentro del grid');
                return;
            }
            
            // Si el modelo ya está inicializado, añadir estación al backend
            if (startButton.disabled === false) {
                fetch('/add_charging_station', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        x: x,
                        y: y
                    })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        chargingStations = data.charging_stations;
                        updateGrid();
                    } else {
                        alert(data.error || 'No se pudo añadir la estación de carga');
                    }
                })
                .catch(error => {
                    console.error('Error:', error);
                    alert('Error al comunicarse con el servidor');
                });
            } else {
                // Si el modelo no está inicializado, solo añadirlo a la lista local
                chargingStations.push({x: x, y: y});
            }
        }
        
        // Función para inicializar el grid
        function initializeGrid() {
            // Obtener valores de los inputs
            gridWidth = parseInt(document.getElementById('gridWidth').value);
            gridHeight = parseInt(document.getElementById('gridHeight').value);
            
            // Validar que hay al menos un robot
            if (robots.length === 0) {
                alert('Añade al menos un robot antes de inicializar');
                return;
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
                    robots: robots.map(robot => ({
                        start: [robot.start.x, robot.start.y],
                        goal: [robot.goal.x, robot.goal.y],
                        color: robot.color,
                        max_battery: robot.max_battery,
                        battery_drain_rate: robot.battery_drain_rate,
                        battery_level: robot.battery_level
                    })),
                    charging_stations: chargingStations.map(station => [station.x, station.y])
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    // Actualizar información desde el backend
                    robots = data.robots;
                    chargingStations = data.charging_stations;
                    obstacles = [];
                    
                    // Actualizar la UI
                    updateGrid();
                    updateStatus('Modelo inicializado. Listo para iniciar la simulación.');
                    startButton.disabled = false;
                    stepButton.disabled = false;
                    resetButton.disabled = false;
                    exportCoordsButton.disabled = false;
                }
            })
            .catch(error => {
                console.error('Error:', error);
                updateStatus('Error al inicializar el modelo.');
            });
        }

        // Reiniciar la simulación
        function resetSimulation() {
            stopSimulation();
            initializeGrid();
        }
        
        // Función para actualizar el grid
        function updateGrid() {
            // Limpiar el grid actual
            gridElement.innerHTML = '';
            robotsContainer.innerHTML = '';
            
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
                    
                    // Verificar si es una estación de carga
                    const isChargingStation = chargingStations.some(s => s.x === x && s.y === y);
                    if (isChargingStation) {
                        cell.classList.add('charging-station');
                    }
                    
                    // Marcar celdas de inicio y meta para cada robot
                    let isStartOrGoal = false;
                    for (const robot of robots) {
                        if (x === robot.start.x && y === robot.start.y) {
                            cell.classList.add('start');
                            cell.textContent = 'S' + robot.id;
                            isStartOrGoal = true;
                            break;
                        } else if (x === robot.goal.x && y === robot.goal.y) {
                            cell.classList.add('goal');
                            cell.textContent = 'G' + robot.id;
                            isStartOrGoal = true;
                            break;
                        }
                    }
                    
                    // Verificar si es un obstáculo
                    const isObstacle = obstacles.some(o => o.x === x && o.y === y);
                    if (isObstacle) {
                        cell.classList.add('obstacle');
                    }
                    
                    // Añadir evento de clic para agregar obstáculos
                    cell.addEventListener('click', (e) => {
                        const cellX = parseInt(e.target.dataset.x);
                        const cellY = parseInt(e.target.dataset.y);
                        
                        // No permitir obstáculos en ciertas posiciones
                        let isReservedPosition = false;
                        
                        // Verificar si es inicio o meta de algún robot
                        for (const robot of robots) {
                            if ((cellX === robot.start.x && cellY === robot.start.y) ||
                                (cellX === robot.goal.x && cellY === robot.goal.y) ||
                                (cellX === robot.position.x && cellY === robot.position.y)) {
                                isReservedPosition = true;
                                break;
                            }
                        }
                        
                        // Verificar si es una estación de carga
                        if (chargingStations.some(s => s.x === cellX && s.y === cellY)) {
                            isReservedPosition = true;
                        }
                        
                        if (isReservedPosition) {
                            return;
                        }
                        
                        // Verificar si ya hay un obstáculo
                        const isObstacle = obstacles.some(o => o.x === cellX && o.y === cellY);
                        
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
                                    obstacles = data.obstacles;
                                    
                                    // Actualizar rutas de los robots
                                    for (const robotPath of data.robots_paths) {
                                        const robot = robots.find(r => r.id === robotPath.id);
                                        if (robot) {
                                            robot.path = robotPath.path;
                                        }
                                    }
                                    
                                    updateGrid();
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
            updatePaths();
        }
        
        // Función para actualizar las rutas en el grid
        function updatePaths() {
            // Limpiar las rutas anteriores
            document.querySelectorAll('.cell.path').forEach(cell => {
                cell.classList.remove('path');
            });
            
            // Para cada robot, mostrar su ruta
            robots.forEach(robot => {
                if (!robot.path || robot.reached_goal) return;
                
                robot.path.forEach(pos => {
                    // No mostrar el camino en inicio, meta o posición actual del robot
                    if ((pos.x === robot.start.x && pos.y === robot.start.y) ||
                        (pos.x === robot.goal.x && pos.y === robot.goal.y) ||
                        (pos.x === robot.position.x && pos.y === robot.position.y)) {
                        return;
                    }
                    
                    // Convertir coordenadas a índice de celda
                    const cellIndex = pos.y * gridWidth + pos.x;
                    const cell = gridElement.children[cellIndex];
                    
                    // Marcar la celda como parte del camino si no es un obstáculo o estación de carga
                    if (cell && !cell.classList.contains('obstacle') && 
                        !cell.classList.contains('charging-station') &&
                        !cell.classList.contains('start') && 
                        !cell.classList.contains('goal')) {
                        cell.classList.add('path');
                    }
                });
            });
        }
        
        // Crear elementos para los robots
        function createRobotElements() {
            robotsContainer.innerHTML = '';
            
            const cellWidth = gridElement.offsetWidth / gridWidth;
            const cellHeight = gridElement.offsetHeight / gridHeight;
            
            robots.forEach(robot => {
                // Crear el elemento del robot
                const robotElement = document.createElement('div');
                robotElement.id = `robot-${robot.id}`;
                robotElement.classList.add('robot');
                robotElement.style.width = `${cellWidth * 0.8}px`;
                robotElement.style.height = `${cellHeight * 0.8}px`;
                robotElement.style.backgroundColor = robot.color;
                robotElement.style.left = `${robot.position.x * cellWidth + (cellWidth * 0.1)}px`;
                robotElement.style.top = `${robot.position.y * cellHeight + (cellHeight * 0.1)}px`;
                
                // Texto del robot (ID)
                robotElement.textContent = robot.id;
                
                // Indicador de batería
                const batteryIndicator = document.createElement('div');
                batteryIndicator.classList.add('battery-indicator');
                
                const batteryLevel = document.createElement('div');
                batteryLevel.classList.add('battery-level');
                
                // Calcular porcentaje de batería
                const batteryPercentage = (robot.battery_level / robot.max_battery) * 100;
                batteryLevel.style.width = `${batteryPercentage}%`;
                
                // Cambiar color si batería baja
                if (batteryPercentage < 30) {
                    batteryLevel.classList.add('battery-low');
                }
                
                batteryIndicator.appendChild(batteryLevel);
                robotElement.appendChild(batteryIndicator);
                
                // Icono de carga si está cargando
                if (robot.charging) {
                    const chargingIcon = document.createElement('div');
                    chargingIcon.classList.add('charging-icon');
                    chargingIcon.textContent = '⚡';
                    robotElement.appendChild(chargingIcon);
                }
                
                robotsContainer.appendChild(robotElement);
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
                            robot.battery_level = robotUpdate.battery_level;
                            robot.charging = robotUpdate.charging;
                            robot.path = robotUpdate.path; // Actualizar la ruta
                        }
                    });
                    
                    // Actualizar la UI
                    createRobotElements();
                    updatePaths(); // Añadido: actualizar las rutas
                    
                    if (data.all_reached_goal) {
                        updateStatus('¡Todos los robots han alcanzado sus metas!');
                        stopSimulation();
                    } else {
                        const robotsInProgress = robots.filter(r => !r.reached_goal).length;
                        updateStatus(`${robotsInProgress} robots en movimiento. ${robots.length - robotsInProgress} han llegado a la meta.`);
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
        
        // Event listeners para los botones
        initButton.addEventListener('click', initializeGrid);
        startButton.addEventListener('click', startSimulation);
        stepButton.addEventListener('click', step);
        resetButton.addEventListener('click', resetSimulation);
        exportCoordsButton.addEventListener('click', () => {
            window.location.href = '/export_path_coordinates';
        });
        addRobotButton.addEventListener('click', addRobot);
        addStationButton.addEventListener('click', addChargingStation);
        
        // Inicializar al cargar la página
        window.addEventListener('load', () => {
            // Añadir un robot por defecto
            addRobot();
            // Añadir una estación de carga por defecto
            addChargingStation();
        });
    </script>
</body>
</html>""")
    
    print("Servidor Flask iniciado en http://127.0.0.1:5000")
    app.run(debug=True)