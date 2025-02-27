import subprocess
import sys

def downgrade_mesa():
    print("Desinstalando Mesa actual...")
    subprocess.run([sys.executable, "-m", "pip", "uninstall", "-y", "mesa"])
    
    print("\nInstalando Mesa 0.9.0...")
    subprocess.run([sys.executable, "-m", "pip", "install", "mesa==0.9.0"])
    
    print("\nVerificando instalación...")
    try:
        import mesa
        print(f"Mesa versión instalada: {mesa.__version__}")
        
        # Verificar módulos clave
        from mesa.visualization.modules import CanvasGrid
        from mesa.visualization.ModularVisualization import ModularServer
        print("Módulos necesarios encontrados correctamente.")
        print("\nLa instalación fue exitosa. Ahora puedes ejecutar el servidor original.")
        
    except ImportError as e:
        print(f"Error al verificar la instalación: {e}")
        print("La instalación no fue completamente exitosa.")

if __name__ == "__main__":
    downgrade_mesa()