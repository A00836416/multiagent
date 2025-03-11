import json
import os

# Función para convertir datos y escribir archivos
def convertir_coordenadas_drones(archivo_entrada, archivo_salida_drone1, archivo_salida_drone2):
    # Leer el archivo JSON
    with open(archivo_entrada, 'r') as f:
        data = json.load(f)
    
    # Extraer coordenadas de los drones
    coordenadas_drone1 = data["drone_1"]
    coordenadas_drone2 = data["drone_2"]
    
    # Escribir archivo para drone 1
    with open(archivo_salida_drone1, 'w') as f:
        f.write(f"# Coordenadas del drone 1\n")
        f.write(f"# Total de puntos: {len(coordenadas_drone1)}\n")
        
        for punto in coordenadas_drone1:
            # Formato: x,y,z con z=0
            f.write(f"{punto[0]},{punto[1]},0\n")
    
    # Escribir archivo para drone 2
    with open(archivo_salida_drone2, 'w') as f:
        f.write(f"# Coordenadas del drone 2\n")
        f.write(f"# Total de puntos: {len(coordenadas_drone2)}\n")
        
        for punto in coordenadas_drone2:
            # Formato: x,y,z con z=0
            f.write(f"{punto[0]},{punto[1]},0\n")
    
    print(f"Se han creado los archivos {os.path.basename(archivo_salida_drone1)} y {os.path.basename(archivo_salida_drone2)}")

# Obtener el directorio actual del script
directorio_script = os.path.dirname(os.path.abspath(__file__))

# Rutas de archivos usando el directorio actual
archivo_entrada = os.path.join(directorio_script, "TargetPositions.txt")
archivo_salida_drone1 = os.path.join(directorio_script, "ruta_drone1.txt")
archivo_salida_drone2 = os.path.join(directorio_script, "ruta_drone2.txt")

# Ejecutar la conversión
try:
    convertir_coordenadas_drones(archivo_entrada, archivo_salida_drone1, archivo_salida_drone2)
except Exception as e:
    print(f"Error al procesar los archivos: {e}")
    print(f"Asegúrate de que el archivo {archivo_entrada} existe y tiene el formato correcto.") 