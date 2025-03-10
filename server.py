#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# IMPORTANTE: Monkey patch primero, antes de cualquier otra importación
import eventlet
eventlet.monkey_patch()

# Ahora importar el resto de módulos
import os
import sys
import time
import tempfile
import datetime
import json
import random
from flask import Flask, render_template, request, send_file
from flask_socketio import SocketIO, emit
from pathfinding_model import RobotAgent, ObstacleAgent, ChargingStation, PathFindingModel


# Crear la aplicación Flask
app = Flask(__name__)
app.config['SECRET_KEY'] = 'robotsimulation2025'  # Clave secreta para sesiones
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# Modelo global para mantener el estado
model = None
obstacles = []
charging_stations = []
robots_config = []

# Definir las posiciones de los camiones y puntos de entrega
truck_positions = [
    (11, 21), (12, 21), (13, 21), 
    (26, 21), (27, 21), (28, 21)
]

delivery_positions = [
    (2, 14), (2, 16), (2, 18),
    (3, 14), (3, 16), (3, 18),
    (5, 14), (5, 16), (5, 18),
    (6, 14), (6, 16), (6, 18),
    (10, 2), (10, 3), (10, 4), (10, 5), (10, 6), (10, 7),
    (13, 2), (13, 3), (13, 4), (13, 5), (13, 6), (13, 7),
    (16, 2), (16, 3), (16, 4), (16, 5), (16, 6), (16, 7),
    (32, 14), (32, 16), (32, 18),
    (33, 14), (33, 16), (33, 18),
    (35, 14), (35, 16), (35, 18),
    (36, 14), (36, 16), (36, 18)
]

# Función para modificar el método deliver_package de RobotAgent
def modify_robot_deliver_package():
    """Modifica el método deliver_package de RobotAgent para añadir asignación automática"""
    original_deliver_package = RobotAgent.deliver_package
    
    def new_deliver_package(self):
        result = original_deliver_package(self)
        if result:
            # El paquete fue entregado con éxito, intentar asignar otro
            # Usamos eventlet.spawn para ejecutar esto de forma asíncrona
            eventlet.spawn(assign_packages_to_available_robots)
        return result
    
    # Reemplazar el método original con el nuevo
    RobotAgent.deliver_package = new_deliver_package
    print("Se ha modificado el método deliver_package para asignación automática")

# Implementar métodos para el modelo
def get_truck_positions(model):
    return truck_positions
    
def get_delivery_positions(model):
    return delivery_positions

# Vincula estos métodos al modelo
PathFindingModel.get_truck_positions = get_truck_positions
PathFindingModel.get_delivery_positions = get_delivery_positions

# Rutas HTTP básicas
@app.route('/')
def index():
    """Ruta principal que muestra la interfaz web"""
    return render_template('index.html')

@app.route('/export_path_coordinates', methods=['GET'])
def export_path_coordinates():
    """Exporta las coordenadas de las rutas de robots en un formato específico (se mantiene como endpoint HTTP)"""
    global model
    
    if model is None:
        return "Modelo no inicializado", 400
    
    output = []
    
    # Procesar cada robot
    for robot in model.robots:
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

# Eventos de WebSocket
@socketio.on('connect')
def handle_connect():
    """Maneja la conexión de un cliente WebSocket"""
    print('Cliente conectado con sid:', request.sid)
    print('Cliente conectado')
    # Enviar estado actual si el modelo ya está inicializado
    if model:
        emit_state()

@socketio.on('disconnect')
def handle_disconnect():
    """Maneja la desconexión de un cliente WebSocket"""
    print('Cliente desconectado')

@socketio.on('initialize')
def handle_initialize(data):
    """Inicializa o reinicia el modelo con parámetros dados"""
    global model, obstacles, charging_stations, robots_config
    
    width = int(data.get('width', 10))
    height = int(data.get('height', 10))
    robots_config = [
        # Robots cerca de las estaciones de carga (parte superior derecha)
        {
            'start': [33, 2],  # Cerca de la estación en (34, 1)
            'goal': [33, 2],   # Mismo punto para que comience en idle
            'color': "blue", 
            'max_battery': 100,
            'battery_drain_rate': 0.5,
            'battery_level': 100,
            'idle': True       # El robot comienza en modo idle
        },
        {
            'start': [37, 4],  # Cerca de la estación en (38, 3)
            'goal': [37, 4],   # Mismo punto para que comience en idle
            'color': "green",
            'max_battery': 100,
            'battery_drain_rate': 0.5,
            'battery_level': 100,
            'idle': True
        },
        # Robots cerca de los camiones (parte inferior)
        {
            'start': [10, 20], # Cerca del camión en (11, 21)
            'goal': [10, 20],  # Mismo punto para que comience en idle
            'color': "red",
            'max_battery': 100,
            'battery_drain_rate': 0.5,
            'battery_level': 100,
            'idle': True
        },
        {
            'start': [26, 20], # Cerca del camión en (27, 21)
            'goal': [26, 20],  # Mismo punto para que comience en idle
            'color': "purple",
            'max_battery': 100,
            'battery_drain_rate': 0.5,
            'battery_level': 100,
            'idle': True
        },
        # Robots en otras posiciones de la parte superior derecha
        {
            'start': [39, 1],  # Esquina superior derecha
            'goal': [39, 1],   # Mismo punto para que comience en idle
            'color': "orange",
            'max_battery': 100,
            'battery_drain_rate': 0.5,
            'battery_level': 100,
            'idle': True
        },
        {
            'start': [38, 10], # Un poco más abajo en la parte derecha
            'goal': [38, 10],  # Mismo punto para que comience en idle
            'color': "cyan",
            'max_battery': 100,
            'battery_drain_rate': 0.5,
            'battery_level': 100,
            'idle': True
        }
    ]
    charging_stations_config = data.get('charging_stations', [])
    obstacles_list = data.get('obstacles', [])  
    
    # Asegurar que todos los robots tengan configuración de inicio y meta
    for i, robot in enumerate(robots_config):
        if 'start' not in robot or 'goal' not in robot:
            emit('error', {'message': f'Configuración incompleta para el robot {i+1}'})
            return
    
    # Inicializar el modelo con múltiples robots y estaciones de carga
    model = PathFindingModel(width, height, robots_config, charging_stations_config)
    
    obstacles = []
    if obstacles_list:
        for obs in obstacles_list:
            # Si el obstáculo está en formato {x:X, y:Y}
            if isinstance(obs, dict):
                x, y = obs.get('x', 0), obs.get('y', 0)
            # Si el obstáculo está en formato [x, y]
            elif isinstance(obs, list):
                x, y = obs[0], obs[1]
            else:
                continue
                
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
            'battery_percentage': (robot.battery_level / robot.max_battery) * 100,
            'idle': getattr(robot, 'idle', False),
            'is_carrying': robot.carrying_package is not None and robot.carrying_package.status == 'picked'
        })
    
    # Generar 2000 paquetes automáticamente después de la inicialización
    generate_initial_packages(2000)
    
    # Emitir evento de inicialización exitosa
    emit('initialization_complete', {
        'grid_size': {'width': width, 'height': height},
        'robots': robots_info,
        'obstacles': obstacles,
        'charging_stations': charging_stations
    })

def generate_initial_packages(count):
    """Genera un número específico de paquetes automáticamente"""
    if model is None:
        return
    
    truck_positions = model.get_truck_positions()
    delivery_positions = model.get_delivery_positions()
    
    print(f"Generando {count} paquetes...")
    packages_created = []
    for _ in range(count):
        truck_pos = random.choice(truck_positions)
        delivery_pos = random.choice(delivery_positions)
        package = model.create_package(truck_pos, delivery_pos)
        packages_created.append({
            'id': package.id,
            'pickup': {'x': package.pickup_location[0], 'y': package.pickup_location[1]},
            'delivery': {'x': package.delivery_location[0], 'y': package.delivery_location[1]},
            'status': package.status
        })
    
    print(f"Se crearon {len(packages_created)} paquetes")
    
    socketio.emit('packages_created', {
        'packages': packages_created[:10],  # Solo enviar los primeros 10 para no sobrecargar la UI
        'total_created': count
    })
    
    # Intentar asignar paquetes a todos los robots inicialmente
    assign_packages_to_available_robots()

def assign_packages_to_available_robots():
    """Asigna paquetes a robots disponibles"""
    if model is None:
        return
    
    # Encontrar robots disponibles (sin paquetes asignados)
    available_robots = [robot for robot in model.robots 
                       if robot.carrying_package is None and 
                       not robot.charging and 
                       getattr(robot, 'idle', False)]
    
    if not available_robots:
        return
    
    # Obtener paquetes disponibles
    available_packages = model.get_available_packages()
    
    if not available_packages:
        return
    
    print(f"Asignando paquetes: {len(available_packages)} disponibles, {len(available_robots)} robots libres")
    
    # Asignar paquetes a robots disponibles
    assignments_made = 0
    for i in range(min(len(available_robots), len(available_packages))):
        robot = available_robots[i]
        package = available_packages[i]
        
        if model.assign_package_to_robot(package.id, robot.unique_id):
            assignments_made += 1
            print(f"Paquete {package.id} asignado al robot {robot.unique_id}")
            
            # Emitir evento de asignación para este paquete específico
            socketio.emit('package_assigned', {
                'package_id': package.id,
                'robot': {
                    'id': robot.unique_id,
                    'goal': {'x': robot.goal[0], 'y': robot.goal[1]},
                    'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path]
                }
            })
    
    if assignments_made > 0:
        print(f"Se asignaron {assignments_made} paquetes")
        # Emitir actualizaciones generales
        emit_robots_update()
        emit_packages_update()

@socketio.on('step')
def handle_step():
    """Avanza un paso en la simulación"""
    global model
    
    if model is None:
        emit('error', {'message': 'Modelo no inicializado'})
        return
    
    # Ejecutar un paso
    model.step()

    # Verificar si hay robots disponibles para asignar paquetes
    assign_packages_to_available_robots()
    
    # Emitir el estado actualizado
    emit_robots_update()
    emit_packages_update()
    
@socketio.on('change_goal')
def handle_change_goal(data):
    """Cambia la meta de un robot"""
    global model
    
    if model is None:
        emit('error', {'message': 'Modelo no inicializado'})
        return
    
    robot_id = int(data.get('robot_id', 0))
    goal_x = int(data.get('goal_x', 0))
    goal_y = int(data.get('goal_y', 0))
    
    # Encontrar el robot
    robot = next((r for r in model.robots if r.unique_id == robot_id), None)
    if not robot:
        emit('error', {'message': f'Robot {robot_id} no encontrado'})
        return
    
    # Cambiar la meta y recalcular la ruta
    success = robot.change_goal((goal_x, goal_y))
    
    if not success:
        emit('error', {'message': 'No se pudo encontrar ruta a la nueva meta'})
        return
    
    emit('goal_changed', {
        'robot_id': robot_id,
        'goal': {'x': goal_x, 'y': goal_y},
        'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path]
    })
    
    # También emitir actualización de estado general
    emit_robots_update()

@socketio.on('add_obstacle')
def handle_add_obstacle(data):
    """Añade un obstáculo en la posición especificada"""
    global model, obstacles
    
    if model is None:
        emit('error', {'message': 'Modelo no inicializado'})
        return
    
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
                emit('error', {'message': f'No se pudo encontrar una ruta para el robot {robot.unique_id} al añadir el obstáculo'})
                return
            
            robots_paths.append({
                'id': robot.unique_id,
                'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path]
            })
        
        emit('obstacle_added', {
            'obstacle': {'x': x, 'y': y},
            'obstacles': obstacles,
            'robots_paths': robots_paths
        })
    else:
        emit('error', {'message': 'No se puede añadir obstáculo en la posición especificada'})

@socketio.on('add_charging_station')
def handle_add_charging_station(data):
    """Añade una estación de carga en la posición especificada"""
    global model, charging_stations
    
    if model is None:
        emit('error', {'message': 'Modelo no inicializado'})
        return
    
    x = int(data.get('x', 0))
    y = int(data.get('y', 0))
    
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
            
        emit('charging_station_added', {
            'charging_station': {'x': x, 'y': y},
            'charging_stations': charging_stations,
            'robots_paths': robots_paths
        })
    else:
        emit('error', {'message': 'No se puede añadir estación de carga en la posición especificada'})

@socketio.on('add_robot')
def handle_robot_added(data):
    """Añade un nuevo robot a la simulación"""
    global model, robots_config
    
    if model is None:
        emit('error', {'message': 'Modelo no inicializado'})
        return
    
    start_x = int(data.get('start_x', 0))
    start_y = int(data.get('start_y', 0))
    goal_x = int(data.get('goal_x', start_x))  # Default to start if not specified
    goal_y = int(data.get('goal_y', start_y))  # Default to start if not specified
    color = data.get('color', "blue")  # Color por defecto
    idle = data.get('idle', True)  # Por defecto, el robot está en idle
    
    # Parámetros de batería
    max_battery = float(data.get('max_battery', 100))
    battery_level = float(data.get('battery_level', max_battery))
    battery_drain_rate = float(data.get('battery_drain_rate', 1))
    
    start = (start_x, start_y)  # Usar tupla
    goal = (goal_x, goal_y)  # Usar tupla
    
    # Verificar que las posiciones no estén ocupadas por obstáculos
    if model.has_obstacle(start) or model.has_obstacle(goal):
        emit('error', {'message': 'Las posiciones de inicio o meta están ocupadas por obstáculos'})
        return
    
    # Crear nuevo robot
    new_robot_id = len(model.robots) + 1
    new_robot = RobotAgent(
        new_robot_id, model, start, goal, color,
        max_battery=max_battery,
        battery_drain_rate=battery_drain_rate,
        battery_level=battery_level
    )
    
    # Establecer estado idle
    new_robot.idle = idle
    
    # Si está en idle, limpiar la ruta
    if idle:
        new_robot.path = []
    
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
        'battery_level': battery_level,
        'idle': idle
    })
    
    emit('robot_added', {
        'robot': {
            'id': new_robot.unique_id,
            'start': {'x': start[0], 'y': start[1]},
            'goal': {'x': goal[0], 'y': goal[1]},
            'position': {'x': start[0], 'y': start[1]},
            'path': [{'x': pos[0], 'y': pos[1]} for pos in new_robot.path] if new_robot.path else [],
            'color': color,
            'battery_level': battery_level,
            'max_battery': max_battery,
            'charging': False,
            'battery_percentage': (battery_level / max_battery) * 100,
            'idle': idle,
            'is_carrying': False  # Un robot nuevo nunca está llevando un paquete
        }
    })

@socketio.on('create_package')
def handle_create_package():
    """Crea un nuevo paquete"""
    global model
    
    if model is None:
        emit('error', {'message': 'Modelo no inicializado'})
        return
    
    # Si no se especifican ubicaciones, seleccionar aleatoriamente
    truck_pos = random.choice(truck_positions)
    delivery_pos = random.choice(delivery_positions)
    
    package = model.create_package(truck_pos, delivery_pos)
    
    emit('package_created', {
        'package': {
            'id': package.id,
            'pickup': {'x': package.pickup_location[0], 'y': package.pickup_location[1]},
            'delivery': {'x': package.delivery_location[0], 'y': package.delivery_location[1]},
            'status': package.status
        }
    })
    
    # También emitir el estado actualizado de paquetes
    emit_packages_update()

@socketio.on('create_packages')
def handle_create_packages(data):
    """Crea múltiples paquetes"""
    global model
    
    if model is None:
        emit('error', {'message': 'Modelo no inicializado'})
        return
    
    count = int(data.get('count', 1))
    
    packages = []
    for _ in range(count):
        truck_pos = random.choice(truck_positions)
        delivery_pos = random.choice(delivery_positions)
        package = model.create_package(truck_pos, delivery_pos)
        packages.append({
            'id': package.id,
            'pickup': {'x': package.pickup_location[0], 'y': package.pickup_location[1]},
            'delivery': {'x': package.delivery_location[0], 'y': package.delivery_location[1]},
            'status': package.status
        })
    
    emit('packages_created', {
        'packages': packages
    })
    
    # También emitir el estado actualizado de paquetes
    emit_packages_update()

@socketio.on('assign_package')
def handle_assign_package(data):
    """Asigna un paquete a un robot"""
    global model
    
    if model is None:
        emit('error', {'message': 'Modelo no inicializado'})
        return
    
    package_id = int(data.get('package_id', 0))
    robot_id = int(data.get('robot_id', 0))
    
    success = model.assign_package_to_robot(package_id, robot_id)
    
    if not success:
        emit('error', {'message': 'No se pudo asignar el paquete'})
        return
    
    robot = next((r for r in model.robots if r.unique_id == robot_id), None)
    
    emit('package_assigned', {
        'package_id': package_id,
        'robot': {
            'id': robot.unique_id,
            'goal': {'x': robot.goal[0], 'y': robot.goal[1]},
            'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path]
        }
    })
    
    # También emitir actualizaciones generales
    emit_robots_update()
    emit_packages_update()

@socketio.on('get_packages')
def handle_get_packages():
    """Retorna información sobre los paquetes"""
    emit_packages_update()

@socketio.on('get_state')
def handle_get_state():
    """Devuelve el estado actual del modelo"""
    emit_state()

@app.route('/get_state', methods=['GET'])
def get_state():
    """Devuelve el estado actual del modelo en formato JSON"""
    global model, obstacles, charging_stations

    if model is None:
        return {"error": "Modelo no inicializado"}, 400

    # Obtener información de todos los robots
    robots_info = []
    for robot in model.robots:
        robot_data = {
            'id': robot.unique_id,
            'start': {'x': robot.start[0], 'y': robot.start[1]},
            'goal': {'x': robot.goal[0], 'y': robot.goal[1]},
            'position': {'x': robot.pos[0], 'y': robot.pos[1]},
            'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path],
            'reached_goal': robot.reached_goal,
            'steps_left': len(robot.path) - 1 if robot.path else 0,
            'steps_taken': robot.steps_taken,
            'color': robot.color,
            'battery_level': robot.battery_level,
            'max_battery': robot.max_battery,
            'charging': robot.charging,
            'battery_percentage': (robot.battery_level / robot.max_battery) * 100,
            'total_packages_delivered': robot.total_packages_delivered,
            'idle': getattr(robot, 'idle', False),
            'is_carrying': robot.carrying_package is not None and robot.carrying_package.status == 'picked',
            'status': 'charging' if robot.charging else 'goal_reached' if robot.reached_goal else 'moving'
        }

        # Añadir información del paquete si el robot lo lleva
        if robot.carrying_package:
            robot_data['carrying_package'] = {
                'id': robot.carrying_package.id,
                'status': robot.carrying_package.status,
                'pickup': {'x': robot.carrying_package.pickup_location[0], 'y': robot.carrying_package.pickup_location[1]},
                'delivery': {'x': robot.carrying_package.delivery_location[0], 'y': robot.carrying_package.delivery_location[1]}
            }

        robots_info.append(robot_data)

    # Obtener información de paquetes activos y entregados
    active_packages = []
    delivered_packages = []
    
    for package in model.packages:
        if package.status != 'delivered':
            active_packages.append({
                'id': package.id,
                'pickup': {'x': package.pickup_location[0], 'y': package.pickup_location[1]},
                'delivery': {'x': package.delivery_location[0], 'y': package.delivery_location[1]},
                'status': package.status,
                'assigned_robot_id': package.assigned_robot_id
            })
    
    for package in model.delivered_packages:
        delivered_packages.append({
            'id': package.id,
            'pickup': {'x': package.pickup_location[0], 'y': package.pickup_location[1]},
            'delivery': {'x': package.delivery_location[0], 'y': package.delivery_location[1]},
            'status': package.status,
            'assigned_robot_id': package.assigned_robot_id,
            'pickup_time': package.pickup_time,
            'delivery_time': package.delivery_time
        })
    
    # Calcular estadísticas de paquetes entregados
    delivered_packages_stats = {}
    if delivered_packages:
        delivery_times = []
        for package in delivered_packages:
            if package.get('pickup_time') is not None and package.get('delivery_time') is not None:
                delivery_time = package['delivery_time'] - package['pickup_time']
                delivery_times.append(delivery_time)
        
        if delivery_times:
            delivered_packages_stats['avg_delivery_time'] = sum(delivery_times) / len(delivery_times)
            delivered_packages_stats['min_delivery_time'] = min(delivery_times)
            delivered_packages_stats['max_delivery_time'] = max(delivery_times)

    # Retornar el estado en formato JSON
    return json.dumps({
        'grid_size': {'width': model.grid.width, 'height': model.grid.height},
        'robots': robots_info,
        'obstacles': obstacles,
        'charging_stations': charging_stations,
        'all_reached_goal': model.all_robots_reached_goal(),
        'total_packages_delivered': len(model.delivered_packages),
        'active_packages': active_packages,
        'delivered_packages': delivered_packages,
        'delivered_packages_stats': delivered_packages_stats
    }, indent=4), 200


# Funciones de emisión de eventos
def emit_state():
    """Emite el estado completo del modelo"""
    global model, obstacles, charging_stations
    
    if model is None:
        return
    
    # Obtener información de todos los robots
    robots_info = []
    for robot in model.robots:
        robot_data = {
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
            'battery_percentage': (robot.battery_level / robot.max_battery) * 100,
            'total_packages_delivered': robot.total_packages_delivered,
            'idle': getattr(robot, 'idle', False),
            'is_carrying': robot.carrying_package is not None and robot.carrying_package.status == 'picked',
            'status': 'charging' if robot.charging else 'goal_reached' if robot.reached_goal else 'moving'
        }
        
        # Añadir información del paquete si lo lleva
        if robot.carrying_package:
            robot_data['carrying_package'] = {
                'id': robot.carrying_package.id,
                'status': robot.carrying_package.status,
                'pickup': {'x': robot.carrying_package.pickup_location[0], 
                          'y': robot.carrying_package.pickup_location[1]},
                'delivery': {'x': robot.carrying_package.delivery_location[0], 
                           'y': robot.carrying_package.delivery_location[1]}
            }
        
        robots_info.append(robot_data)
    
    # Calcular estadísticas de paquetes entregados
    delivered_packages_stats = {}
    if model.delivered_packages:
        delivery_times = []
        for package in model.delivered_packages:
            if package.pickup_time is not None and package.delivery_time is not None:
                delivery_time = package.delivery_time - package.pickup_time
                delivery_times.append(delivery_time)
        
        if delivery_times:
            delivered_packages_stats['avg_delivery_time'] = sum(delivery_times) / len(delivery_times)
            delivered_packages_stats['min_delivery_time'] = min(delivery_times)
            delivered_packages_stats['max_delivery_time'] = max(delivery_times)
    
    socketio.emit('state_update', {
        'grid_size': {'width': model.grid.width, 'height': model.grid.height},
        'robots': robots_info,
        'obstacles': obstacles,
        'charging_stations': charging_stations,
        'all_reached_goal': model.all_robots_reached_goal(),
        'total_packages_delivered': len(model.delivered_packages),
        'active_packages': len([p for p in model.packages if p.status != 'delivered']),
        'delivered_packages_stats': delivered_packages_stats
    })

def emit_robots_update():
    """Emite información actualizada de todos los robots"""
    global model
    
    if model is None:
        return
    
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
            'path': [{'x': pos[0], 'y': pos[1]} for pos in robot.path],  # Incluir la ruta actualizada
            'idle': getattr(robot, 'idle', False),  # Atributo idle
            'is_carrying': robot.carrying_package is not None and robot.carrying_package.status == 'picked'
        })
    
    socketio.emit('robots_update', {
        'robots': robots_info,
        'all_reached_goal': all_reached_goal
    })

def emit_packages_update():
    """Emite información actualizada de los paquetes"""
    global model
    
    if model is None:
        return
    
    active_packages = []
    delivered_packages = []
    
    for package in model.packages:
        if package.status != 'delivered':
            active_packages.append({
                'id': package.id,
                'pickup': {'x': package.pickup_location[0], 'y': package.pickup_location[1]},
                'delivery': {'x': package.delivery_location[0], 'y': package.delivery_location[1]},
                'status': package.status,
                'assigned_robot_id': package.assigned_robot_id
            })
    
    for package in model.delivered_packages:
        delivered_packages.append({
            'id': package.id,
            'pickup': {'x': package.pickup_location[0], 'y': package.pickup_location[1]},
            'delivery': {'x': package.delivery_location[0], 'y': package.delivery_location[1]},
            'status': package.status,
            'assigned_robot_id': package.assigned_robot_id,
            'pickup_time': package.pickup_time,
            'delivery_time': package.delivery_time
        })
    
    socketio.emit('packages_update', {
        'active_packages': active_packages,
        'delivered_packages': delivered_packages,
        'total_delivered': len(delivered_packages)
    })

# Función para la automatización de pasos
def run_simulation_step():
    """Ejecuta un paso de la simulación automática"""
    global model
    
    if model is None or model.all_robots_reached_goal():
        return False
        
    # Ejecutar un paso
    model.step()
    
    # Asignar paquetes a robots disponibles
    assign_packages_to_available_robots()
    
    # Emitir actualizaciones
    emit_robots_update()
    emit_packages_update()
    
    return True

# Eventos para control de simulación automática
@socketio.on('start_simulation')
def handle_start_simulation():
    """Inicia la simulación automática"""
    if model is None:
        emit('error', {'message': 'Modelo no inicializado'})
        return
    
    # Usar un enfoque simplificado que funciona mejor con eventlet
    # En lugar de background_task, usamos un bucle que se ejecuta directamente
    def simulation_loop():
        while True:
            if not run_simulation_step():
                # Si los robots alcanzaron su meta, detener simulación
                socketio.emit('simulation_stopped', {'reason': 'completed'})
                break
            eventlet.sleep(0.2)  
    
    # Iniciar la simulación en un green thread de eventlet
    eventlet.spawn(simulation_loop)
    emit('simulation_started', {'message': 'Simulación iniciada'})

@socketio.on('stop_simulation')
def handle_stop_simulation():
    """Detiene la simulación automática"""
    # La simulación se detendrá cuando el cliente se desconecte o
    # cuando los robots alcancen su meta
    emit('simulation_stopped', {'reason': 'user_request'})

# Código para generar la plantilla HTML y ejecutar el servidor
if __name__ == '__main__':
    # Crear el directorio templates si no existe
    if not os.path.exists('templates'):
        os.makedirs('templates')

    # Aplicar el parche para la asignación automática de paquetes
    modify_robot_deliver_package()
    
    # Generar la interfaz HTML con funciones WebSocket
    with open('templates/index.html', 'w') as f:
        f.write("""<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Simulación de Robots con WebSockets</title>
    <!-- Agregar Socket.IO -->
    <script src="https://cdn.socket.io/4.6.0/socket.io.min.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 1300px;
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
            display: flex;
            flex-direction: column;
            position: relative;
        }
        .grid-row {
            display: flex;
            flex-direction: row-reverse;
            width: 100%;
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
            aspect-ratio: 1;
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

        .truck-position {
            background-color: #795548 !important; /* Marrón para representar camiones */
            color: white;
            font-size: 1.2em;
        }

        .delivery-point {
            background-color: #8BC34A !important; /* Verde para puntos de entrega */
            color: white;
            font-size: 1.2em;
        }
        
        #exportCoordsBtn {
            background-color: #9c27b0;  /* Color morado para diferenciarlo */
        }
        #exportCoordsBtn:hover {
            background-color: #7b1fa2;
        }
        
        #robots-container {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            pointer-events: none;
            z-index: 100;
        }

        .delivery-point {
        background-color: #8BC34A !important; /* Verde para puntos de entrega */
        color: white;
        font-size: 1.2em;
        }  
    
        .packages-container {
            margin-top: 20px;
            border: 1px solid #ddd;
            padding: 10px;
            border-radius: 4px;
        }
    
        .package {
            background-color: #f5f5f5;
            padding: 10px;
            margin-bottom: 10px;
            border-radius: 4px;
            border-left: 4px solid #FFC107;
        }
        
        .package.assigned {
            border-left-color: #2196F3;
        }
        
        .package.picked {
            border-left-color: #9C27B0;
        }
        
        .stats-container {
            display: flex;
            gap: 20px;
            margin-top: 20px;
            padding: 10px;
            background-color: #f0f0f0;
            border-radius: 4px;
        }
        
        .stat-item {
            flex: 1;
            text-align: center;
            padding: 10px;
            background-color: white;
            border-radius: 4px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        
        .stat-value {
            font-size: 24px;
            font-weight: bold;
            color: #2196F3;
        }
        
        .robot-carrying {
            position: relative;
        }

        .robot.idle {
            opacity: 0.8;
            border: 3px dashed #FF0000;
            box-shadow: 0 0 5px rgba(0, 0, 0, 0.5);
        }
        
        .robot-carrying::after {
            content: "📦";
            position: absolute;
            top: -15px;
            right: -10px;
            font-size: 12px;
        }

        .connection-status {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 5px 10px;
            border-radius: 4px;
            font-weight: bold;
        }
        
        .connected {
            background-color: #4CAF50;
            color: white;
        }
        
        .disconnected {
            background-color: #f44336;
            color: white;
        }
    </style>
</head>
<body>
    <div id="connection-status" class="connection-status disconnected">Desconectado</div>
    <div class="container">
        <h1>Simulación de Robots</h1>
        
        <div class="controls">
            <div class="control-group">
                <div class="control-item">
                    <label for="gridWidth">Ancho del Grid:</label>
                    <input type="number" id="gridWidth" min="5" max="50" value="40">
                </div>
                <div class="control-item">
                    <label for="gridHeight">Alto del Grid:</label>
                    <input type="number" id="gridHeight" min="5" max="50" value="22">
                </div>
            </div>
        </div>
        
        <div class="controls">
            <div class="control-group">
                <h3>Añadir Robot</h3>
                <div class="control-item">
                    <label for="startX">Inicio X:</label>
                    <input type="number" id="startX" min="0" max="39" value="1">
                </div>
                <div class="control-item">
                    <label for="startY">Inicio Y:</label>
                    <input type="number" id="startY" min="0" max="21" value="1">
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
                    <input type="number" id="stationX" min="0" max="39" value="20">
                </div>
                <div class="control-item">
                    <label for="stationY">Posición Y:</label>
                    <input type="number" id="stationY" min="0" max="21" value="10">
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

        <div class="stats-container">
            <div class="stat-item">
                <div>Paquetes Entregados</div>
                <div class="stat-value" id="total-delivered">0</div>
            </div>
            <div class="stat-item">
                <div>Paquetes Activos</div>
                <div class="stat-value" id="active-packages">0</div>
            </div>
        </div>

        <div>
            <h2>Sistema de Paquetes Automático</h2>
            <p>El sistema genera 2000 paquetes al inicio y asigna automáticamente nuevos paquetes a cada robot cuando termina de entregar uno.</p>
            <div class="stats-container">
                <div class="stat-item">
                    <div>Paquetes Disponibles</div>
                    <div class="stat-value" id="available-packages">0</div>
                </div>
                <div class="stat-item">
                    <div>Paquetes Asignados</div>
                    <div class="stat-value" id="assigned-packages">0</div>
                </div>
            </div>
            <div id="packages-container" class="packages-container">
                <p>Cargando paquetes...</p>
            </div>
        </div>
        
        <div class="status" id="status">Estado: No inicializado</div>
    </div>

    <script>
        // Conexión WebSocket
        const socket = io('http://localhost:8080', {
            transports: ['websocket'],
            upgrade: false,
            reconnection: true,
            reconnectionAttempts: 5
        });


        const connectionStatus = document.getElementById('connection-status');
        
        // Variables globales
        let gridWidth = 40;
        let gridHeight = 22;
        let robots = [];
        let obstacles = [];
        let chargingStations = [];
        let isRunning = false;
        let activePackages = [];
        let deliveredPackages = [];
        
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

        const predefinedObstacles = [
            {x: 2, y: 2}, {x: 2, y: 3}, {x: 2, y: 4}, {x: 2, y: 5}, {x: 2, y: 6}, {x: 2, y: 7}, {x: 2, y: 13}, {x: 2, y: 15}, {x: 2, y: 17}, {x: 2, y: 19},
            {x: 3, y: 2}, {x: 3, y: 3}, {x: 3, y: 4}, {x: 3, y: 5}, {x: 3, y: 6}, {x: 3, y: 7}, {x: 3, y: 13}, {x: 3, y: 15}, {x: 3, y: 17}, {x: 3, y: 19},
            {x: 5, y: 2}, {x: 5, y: 3}, {x: 5, y: 4}, {x: 5, y: 5}, {x: 5, y: 6}, {x: 5, y: 7}, {x: 5, y: 13}, {x: 5, y: 15}, {x: 5, y: 17}, {x: 5, y: 19},
            {x: 6, y: 2}, {x: 6, y: 3}, {x: 6, y: 4}, {x: 6, y: 5}, {x: 6, y: 6}, {x: 6, y: 7}, {x: 6, y: 13}, {x: 6, y: 15}, {x: 6, y: 17}, {x: 6, y: 19},
            {x: 11, y: 2}, {x: 11, y: 3}, {x: 11, y: 4}, {x: 11, y: 5}, {x: 11, y: 6}, {x: 11, y: 7},
            {x: 12, y: 2}, {x: 12, y: 3}, {x: 12, y: 4}, {x: 12, y: 5}, {x: 12, y: 6}, {x: 12, y: 7},
            {x: 14, y: 2}, {x: 14, y: 3}, {x: 14, y: 4}, {x: 14, y: 5}, {x: 14, y: 6}, {x: 14, y: 7},
            {x: 15, y: 2}, {x: 15, y: 3}, {x: 15, y: 4}, {x: 15, y: 5}, {x: 15, y: 6}, {x: 15, y: 7},
            {x: 18, y: 11}, {x: 18, y: 13}, {x: 18, y: 15}, {x: 18, y: 17}, {x: 18, y: 19},
            {x: 19, y: 11}, {x: 19, y: 13}, {x: 19, y: 15}, {x: 19, y: 17}, {x: 19, y: 19},
            {x: 20, y: 2}, {x: 20, y: 3}, {x: 20, y: 4}, {x: 20, y: 5}, {x: 20, y: 6}, {x: 20, y: 7},
            {x: 21, y: 2}, {x: 21, y: 3}, {x: 21, y: 4}, {x: 21, y: 5}, {x: 21, y: 6}, {x: 21, y: 7}, {x: 21, y: 11}, {x: 21, y: 13}, {x: 21, y: 15}, {x: 21, y: 17}, {x: 21, y: 19},
            {x: 22, y: 11}, {x: 22, y: 13}, {x: 22, y: 15}, {x: 22, y: 17}, {x: 22, y: 19},
            {x: 23, y: 2}, {x: 23, y: 3}, {x: 23, y: 4}, {x: 23, y: 5}, {x: 23, y: 6}, {x: 23, y: 7},
            {x: 24, y: 2}, {x: 24, y: 3}, {x: 24, y: 4}, {x: 24, y: 5}, {x: 24, y: 6}, {x: 24, y: 7},
            {x: 32, y: 13}, {x: 32, y: 15}, {x: 32, y: 17}, {x: 32, y: 19},
            {x: 33, y: 13}, {x: 33, y: 15}, {x: 33, y: 17}, {x: 33, y: 19},
            {x: 35, y: 13}, {x: 35, y: 15}, {x: 35, y: 17}, {x: 35, y: 19},
            {x: 36, y: 13}, {x: 36, y: 15}, {x: 36, y: 17}, {x: 36, y: 19}
        ];

        const predefinedChargingStations = [
            {x: 34, y: 1},
            {x: 34, y: 3},
            {x: 36, y: 1},
            {x: 36, y: 3},
            {x: 38, y: 1},
            {x: 38, y: 3}
        ];

        // Definir posiciones de camiones para recogida de paquetes
        const predefinedTruckPositions = [
            {x: 11, y: 21},
            {x: 12, y: 21},
            {x: 13, y: 21},
            {x: 26, y: 21},
            {x: 27, y: 21},
            {x: 28, y: 21}
        ];

        // Definir los puntos de entrega (metas)
        const predefinedDeliveryPoints = [
            {x: 2, y: 14}, {x: 2, y: 16}, {x: 2, y: 18},
            {x: 3, y: 14}, {x: 3, y: 16}, {x: 3, y: 18},
            {x: 5, y: 14}, {x: 5, y: 16}, {x: 5, y: 18},
            {x: 6, y: 14}, {x: 6, y: 16}, {x: 6, y: 18},
            {x: 10, y: 2}, {x: 10, y: 3}, {x: 10, y: 4}, {x: 10, y: 5}, {x: 10, y: 6}, {x: 10, y: 7},
            {x: 13, y: 2}, {x: 13, y: 3}, {x: 13, y: 4}, {x: 13, y: 5}, {x: 13, y: 6}, {x: 13, y: 7},
            {x: 16, y: 2}, {x: 16, y: 3}, {x: 16, y: 4}, {x: 16, y: 5}, {x: 16, y: 6}, {x: 16, y: 7},
            {x: 32, y: 14}, {x: 32, y: 16}, {x: 32, y: 18},
            {x: 33, y: 14}, {x: 33, y: 16}, {x: 33, y: 18},
            {x: 35, y: 14}, {x: 35, y: 16}, {x: 35, y: 18},
            {x: 36, y: 14}, {x: 36, y: 16}, {x: 36, y: 18}
        ];
        
        // Eventos de Socket.IO
        socket.on('connect', () => {
            console.log('Conectado al servidor WebSocket');
            connectionStatus.textContent = 'Conectado';
            connectionStatus.className = 'connection-status connected';
            updateStatus('Conectado al servidor. Listo para inicializar.');
        });
        
        socket.on('disconnect', () => {
            console.log('Desconectado del servidor WebSocket');
            connectionStatus.textContent = 'Desconectado';
            connectionStatus.className = 'connection-status disconnected';
            updateStatus('Desconectado del servidor.');
            stopSimulation();
        });
        
        socket.on('error', (data) => {
            console.error('Error:', data.message);
            updateStatus(`Error: ${data.message}`);
        });
        
        socket.on('initialization_complete', (data) => {
            console.log('Inicialización completada:', data);
            robots = data.robots;
            obstacles = data.obstacles;
            chargingStations = data.charging_stations;
            
            updateGrid();
            updateStatus('Modelo inicializado con obstáculos y estaciones de carga predefinidos. Listo para iniciar la simulación.');
            startButton.disabled = false;
            stepButton.disabled = false;
            resetButton.disabled = false;
            exportCoordsButton.disabled = false;
        });
        
        socket.on('robots_update', (data) => {
            // Actualizar la información de los robots
            data.robots.forEach(robotUpdate => {
                const robot = robots.find(r => r.id === robotUpdate.id);
                if (robot) {
                    robot.position = robotUpdate.position;
                    robot.reached_goal = robotUpdate.reached_goal;
                    robot.steps_taken = robotUpdate.steps_taken;
                    robot.battery_level = robotUpdate.battery_level;
                    robot.charging = robotUpdate.charging;
                    robot.path = robotUpdate.path;
                    robot.idle = robotUpdate.idle;
                    robot.is_carrying = robotUpdate.is_carrying;
                }
            });
            
            // Actualizar la UI
            createRobotElements();
            updatePaths();
            
            if (data.all_reached_goal) {
                updateStatus('¡Todos los robots han alcanzado sus metas!');
                stopSimulation();
            } else {
                const robotsInProgress = robots.filter(r => !r.reached_goal).length;
                updateStatus(`${robotsInProgress} robots en movimiento. ${robots.length - robotsInProgress} han llegado a la meta.`);
            }
        });
        
        socket.on('packages_update', (data) => {
            activePackages = data.active_packages;
            deliveredPackages = data.delivered_packages;
            updatePackagesUI();
        });
        
        socket.on('state_update', (data) => {
            // Actualización completa del estado
            robots = data.robots;
            obstacles = data.obstacles;
            chargingStations = data.charging_stations;
            
            // Actualizar todas las vistas
            updateGrid();
            updateStatus(`Total paquetes entregados: ${data.total_packages_delivered}. Activos: ${data.active_packages}`);
            
            // Actualizar contadores
            document.getElementById('total-delivered').textContent = data.total_packages_delivered;
            document.getElementById('active-packages').textContent = data.active_packages;
        });
        
        socket.on('simulation_complete', (data) => {
            updateStatus(data.message);
            stopSimulation();
        });
        
        socket.on('simulation_stopped', (data) => {
            stopSimulation();
            if (data.reason === 'completed') {
                updateStatus('Simulación completada. Todos los robots han alcanzado sus metas.');
            } else {
                updateStatus('Simulación detenida por el usuario.');
            }
        });

        socket.on('obstacle_added', (data) => {
            obstacles = data.obstacles;
            
            // Actualizar rutas de los robots
            data.robots_paths.forEach(robotPath => {
                const robot = robots.find(r => r.id === robotPath.id);
                if (robot) {
                    robot.path = robotPath.path;
                }
            });
            
            updateGrid();
            updatePaths();
        });
        
        socket.on('charging_station_added', (data) => {
            chargingStations = data.charging_stations;
            
            // Actualizar rutas de los robots
            data.robots_paths.forEach(robotPath => {
                const robot = robots.find(r => r.id === robotPath.id);
                if (robot) {
                    robot.path = robotPath.path;
                }
            });
            
            updateGrid();
            updatePaths();
        });
        
        socket.on('robot_added', (data) => {
            robots.push(data.robot);
            updateGrid();
        });
        
        socket.on('package_created', (data) => {
            fetchPackagesStatus();
        });
        
        socket.on('packages_created', (data) => {
            fetchPackagesStatus();
        });
        
        socket.on('package_assigned', (data) => {
            const robot = robots.find(r => r.id === data.robot.id);
            if (robot) {
                robot.goal = data.robot.goal;
                robot.path = data.robot.path;
            }
            fetchPackagesStatus();
        });
        
        // Función para añadir un robot
        function addRobot() {
            const startX = parseInt(document.getElementById('startX').value);
            const startY = parseInt(document.getElementById('startY').value);
            const color = document.getElementById('robotColor').value;
            const maxBattery = parseFloat(document.getElementById('maxBattery').value);
            const batteryDrainRate = parseFloat(document.getElementById('batteryDrainRate').value);
            
            // Ya no necesitamos meta para un robot idle
            const goalX = startX;  // La meta inicial es su propia posición (sin movimiento)
            const goalY = startY;
            
            // Validar que las coordenadas de inicio estén dentro del grid
            if (startX >= gridWidth || startY >= gridHeight || 
                startX < 0 || startY < 0) {
                alert('La posición de inicio debe estar dentro del grid');
                return;
            }
            
            // Si el modelo ya está inicializado, añadir robot al backend
            if (startButton.disabled === false) {
                socket.emit('add_robot', {
                    start_x: startX,
                    start_y: startY,
                    goal_x: goalX,
                    goal_y: goalY,
                    color: color,
                    max_battery: maxBattery,
                    battery_drain_rate: batteryDrainRate,
                    battery_level: maxBattery,
                    idle: true  // Indicar que el robot comienza en estado idle
                });
            } else {
                // Si el modelo no está inicializado, solo añadirlo a la lista local
                const robotId = robots.length + 1;
                robots.push({
                    id: robotId,
                    start: {x: startX, y: startY},
                    goal: {x: startX, y: startY},  // Meta inicial es su propia posición
                    position: {x: startX, y: startY},
                    color: color,
                    max_battery: maxBattery,
                    battery_drain_rate: batteryDrainRate,
                    battery_level: maxBattery,
                    path: [],
                    idle: true
                });
            }
        }

        function isTruckPosition(x, y) {
            return predefinedTruckPositions.some(pos => pos.x === x && pos.y === y);
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
                socket.emit('add_charging_station', {
                    x: x,
                    y: y
                });
            } else {
                // Si el modelo no está inicializado, solo añadirlo a la lista local
                chargingStations.push({x: x, y: y});
            }
        }
        
        function initializeGrid() {
            // Obtener valores de los inputs
            gridWidth = parseInt(document.getElementById('gridWidth').value);
            gridHeight = parseInt(document.getElementById('gridHeight').value);
            
            // Validar que hay al menos un robot
            if (robots.length === 0) {
                alert('Añade al menos un robot antes de inicializar');
                return;
            }
            
            // Filtrar obstáculos que están dentro de los límites del grid
            const validObstacles = predefinedObstacles.filter(
                obs => obs.x < gridWidth && obs.y < gridHeight && obs.x >= 0 && obs.y >= 0
            );
            
            // Filtrar estaciones de carga predefinidas que están dentro de los límites del grid
            const validChargingStations = predefinedChargingStations.filter(
                station => station.x < gridWidth && station.y < gridHeight && station.x >= 0 && station.y >= 0
            );
            
            // Enviar evento de inicialización al servidor WebSocket
            socket.emit('initialize', {
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
                charging_stations: validChargingStations.map(station => [station.x, station.y]),
                obstacles: validObstacles
            });
        }

        // Añadir después de inicializar
        window.addEventListener('load', () => {
            // Añadir un botón al final de la sección de botones
            const buttonsDiv = document.querySelector('.buttons');
            const loadObstaclesBtn = document.createElement('button');
            loadObstaclesBtn.textContent = 'Cargar Obstáculos Predefinidos';
            loadObstaclesBtn.style.backgroundColor = '#FF5722';
            loadObstaclesBtn.onclick = addPredefinedObstacles;
            loadObstaclesBtn.disabled = true;
            buttonsDiv.appendChild(loadObstaclesBtn);
            
            // Habilitar el botón después de inicializar
            document.getElementById('initBtn').addEventListener('click', () => {
                setTimeout(() => {
                    if (!document.getElementById('startBtn').disabled) {
                        loadObstaclesBtn.disabled = false;
                    }
                }, 500);
            });
        });

        // Función para agregar obstáculos predefinidos
        function addPredefinedObstacles() {
            for (const obs of predefinedObstacles) {
                socket.emit('add_obstacle', { x: obs.x, y: obs.y });
            }
        }

        // Reiniciar la simulación
        function resetSimulation() {
            stopSimulation();
            initializeGrid();
        }
        
        // Función para actualizar el grid - MODIFICADA para invertir el orden
        function updateGrid() {
            // Limpiar el grid actual
            gridElement.innerHTML = '';
            robotsContainer.innerHTML = '';
            
            // Crear filas y celdas
            for (let y = 0; y < gridHeight; y++) {
                const rowDiv = document.createElement('div');
                rowDiv.className = 'grid-row';
                
                for (let x = 0; x < gridWidth; x++) {
                    const cell = document.createElement('div');
                    cell.classList.add('cell');
                    cell.dataset.x = x;
                    cell.dataset.y = y;
                    cell.style.width = `${100/gridWidth}%`;
                    
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

                    // Verificar si es un punto de entrega
                    const isDeliveryPoint = predefinedDeliveryPoints.some(d => d.x === x && d.y === y);
                    if (isDeliveryPoint) {
                        cell.classList.add('delivery-point');
                        cell.innerHTML = "📦"; // Emoji de paquete
                    }

                    const isTruckPosition = predefinedTruckPositions.some(t => t.x === x && t.y === y);
                    if (isTruckPosition) {
                        cell.classList.add('truck-position');
                        cell.innerHTML = "🚚"; // Emoji de camión
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
                            // Añadir obstáculo usando WebSocket
                            socket.emit('add_obstacle', { x: cellX, y: cellY });
                        }
                    });
                    
                    rowDiv.appendChild(cell);
                }
                
                gridElement.appendChild(rowDiv);
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
                    
                    // Encontrar la celda correspondiente
                    const cellSelector = `.cell[data-x="${pos.x}"][data-y="${pos.y}"]`;
                    const cell = document.querySelector(cellSelector);
                    
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
            robots.forEach(robot => {
                // Encontrar la celda del robot
                const cellSelector = `.cell[data-x="${robot.position.x}"][data-y="${robot.position.y}"]`;
                const cell = document.querySelector(cellSelector);
                
                if (cell) {
                    const cellRect = cell.getBoundingClientRect();
                    const gridRect = gridElement.getBoundingClientRect();
                    
                    // Crear el elemento del robot
                    const robotElement = document.createElement('div');
                    robotElement.id = `robot-${robot.id}`;
                    robotElement.classList.add('robot');

                    // Asegurarse de que los robots sean visibles incluso en estado idle
                    if (robot.idle) {
                        robotElement.classList.add('idle');
                        // Añadir texto para hacer más visible el robot idle
                        robotElement.setAttribute('title', 'Robot en espera');
                    }
                    
                    // Usar la propiedad is_carrying para determinar si el robot está llevando un paquete
                    if (robot.is_carrying) {
                        robotElement.classList.add('robot-carrying');
                        robotElement.setAttribute('title', 'Robot llevando paquete');
                    }
                    
                    // Tamaño basado en el tamaño de la celda
                    const cellSize = cellRect.width;
                    robotElement.style.width = `${cellSize * 0.8}px`;
                    robotElement.style.height = `${cellSize * 0.8}px`;
                    robotElement.style.backgroundColor = robot.color;
                    
                    // Posicionar el robot
                    const offsetLeft = cellRect.left - gridRect.left;
                    const offsetTop = cellRect.top - gridRect.top;
                    robotElement.style.left = `${offsetLeft + (cellSize * 0.1)}px`;
                    robotElement.style.top = `${offsetTop + (cellSize * 0.1)}px`;
                    
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
                    
                    // Asegurarse de que el robot siempre se añada al contenedor
                    robotsContainer.appendChild(robotElement);
                    
                    // Debug: Añadir un log para verificar que se está creando el robot
                    console.log(`Robot ${robot.id} creado en posición (${robot.position.x}, ${robot.position.y}), idle: ${robot.idle}, carrying: ${robot.is_carrying}`);
                } else {
                    console.error(`No se encontró celda para el robot ${robot.id} en posición (${robot.position.x}, ${robot.position.y})`);
                }
            });
        }

        // Funciones para manejar paquetes
        function createPackages(count) {
            socket.emit('create_packages', {
                count: count
            });
        }

        function assignPackages() {
            // Encontrar robots disponibles (sin paquetes asignados)
            const availableRobots = robots.filter(robot => !robot.carrying_package);
            if (availableRobots.length === 0) {
                alert('No hay robots disponibles para asignar paquetes.');
                return;
            }
            
            // Buscar paquetes por asignar emitiendo un evento
            socket.emit('get_packages');
            
            // El resto se maneja en los eventos: 'packages_update', 'package_assigned'
            socket.once('packages_update', (data) => {
                const unassignedPackages = data.active_packages.filter(p => p.status === 'waiting');
                
                if (unassignedPackages.length === 0) {
                    alert('No hay paquetes pendientes de asignar.');
                    return;
                }
                
                // Asignar paquetes a robots disponibles
                for (let i = 0; i < Math.min(availableRobots.length, unassignedPackages.length); i++) {
                    socket.emit('assign_package', {
                        package_id: unassignedPackages[i].id,
                        robot_id: availableRobots[i].id
                    });
                }
            });
        }

        function fetchPackagesStatus() {
            socket.emit('get_packages');
        }

        function fetchState() {
            socket.emit('get_state');
        }

        function updatePackagesUI() {
            const packagesContainer = document.getElementById('packages-container');
            const availablePackagesCounter = document.getElementById('available-packages');
            const assignedPackagesCounter = document.getElementById('assigned-packages');
            
            if (activePackages.length === 0) {
                packagesContainer.innerHTML = '<p>No hay paquetes pendientes.</p>';
                return;
            }
            
            let html = '';
            let waitingCount = 0;
            let assignedCount = 0;
            let pickedCount = 0;
            
            for (const pkg of activePackages) {
                if (pkg.status === 'waiting') waitingCount++;
                else if (pkg.status === 'assigned') assignedCount++;
                else if (pkg.status === 'picked') pickedCount++;
            }
            
            // Mostrar solo una muestra de los paquetes (máximo 10)
            const packagesToShow = activePackages.slice(0, 10);
            
            for (const pkg of packagesToShow) {
                const statusClass = pkg.status;
                html += `
                    <div class="package ${statusClass}">
                        <h3>Paquete #${pkg.id}</h3>
                        <p>Recogida: (${pkg.pickup.x},${pkg.pickup.y})</p>
                        <p>Entrega: (${pkg.delivery.x},${pkg.delivery.y})</p>
                        <p>Estado: ${getStatusText(pkg.status)}</p>
                        ${pkg.assigned_robot_id ? `<p>Asignado a Robot #${pkg.assigned_robot_id}</p>` : ''}
                    </div>
                `;
            }
            
            if (activePackages.length > 10) {
                html += `<p>... y ${activePackages.length - 10} paquetes más no mostrados.</p>`;
            }
            
            packagesContainer.innerHTML = html;
            
            // Actualizar contadores
            if (availablePackagesCounter) availablePackagesCounter.textContent = waitingCount;
            if (assignedPackagesCounter) assignedPackagesCounter.textContent = assignedCount + pickedCount;
            
            // Actualizar contadores generales
            document.getElementById('total-delivered').textContent = deliveredPackages.length;
            document.getElementById('active-packages').textContent = activePackages.length;
        }

        function getStatusText(status) {
            switch(status) {
                case 'waiting': return 'Esperando asignación';
                case 'assigned': return 'Asignado (en ruta a recoger)';
                case 'picked': return 'Recogido (en ruta a entregar)';
                case 'delivered': return 'Entregado';
                default: return status;
            }
        }
        
        // Actualizar texto de estado
        function updateStatus(message) {
            statusElement.textContent = `Estado: ${message}`;
        }
        
        // Función para ejecutar un paso
        function step() {
            socket.emit('step');
            // Después de cada paso, actualizar el estado de los paquetes
            setTimeout(fetchPackagesStatus, 200);
        }
        
        // Iniciar la simulación automática
        function startSimulation() {
            if (!isRunning) {
                isRunning = true;
                startButton.textContent = 'Detener';
                stepButton.disabled = true;
                
                // Enviar evento para iniciar simulación automática
                socket.emit('start_simulation');
            } else {
                stopSimulation();
            }
        }
        
        // Detener la simulación
        function stopSimulation() {
            isRunning = false;
            startButton.textContent = 'Iniciar';
            stepButton.disabled = false;
            
            // Enviar evento para detener simulación
            socket.emit('stop_simulation');
        }
        
        // Escuchar evento de paquetes iniciales creados
        socket.on('initial_packages_created', (data) => {
            console.log(`Se han creado ${data.total_created} paquetes iniciales`);
            // Actualizar la UI
            fetchPackagesStatus();
        });
        
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
        });
    </script>
</body>
</html>""")
    
    socketio.run(app, debug=True, port=8080, host='0.0.0.0', log_output=True, use_reloader=False)