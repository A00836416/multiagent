import mesa
print("Mesa Version:", mesa.__version__)
print("\nMódulos principales:")
print(dir(mesa))

print("\nVerificando visualization:")
try:
    import mesa.visualization
    print("\nMódulos en mesa.visualization:")
    print(dir(mesa.visualization))
    
    print("\nDetalles de componentes:")
    try:
        import mesa.visualization.components
        print("Módulos en mesa.visualization.components:")
        print(dir(mesa.visualization.components))
    except ImportError:
        print("No se pudo importar mesa.visualization.components")
    
    print("\nVerificando JupyterViz:")
    try:
        from mesa.visualization import JupyterViz
        print("JupyterViz disponible, atributos:")
        print(dir(JupyterViz))
    except ImportError:
        print("No se pudo importar JupyterViz")
    
    print("\nVerificando Slider:")
    try:
        from mesa.visualization import Slider
        print("Slider disponible, atributos:")
        print(dir(Slider))
    except ImportError:
        print("No se pudo importar Slider")
    
except ImportError:
    print("No se pudo importar mesa.visualization")

# Veamos si existe el módulo SolaraViz que mencionaste
print("\nVerificando SolaraViz:")
try:
    from mesa.visualization import SolaraViz
    print("SolaraViz disponible, atributos:")
    print(dir(SolaraViz))
except ImportError:
    print("No se pudo importar SolaraViz")

# También verificar si aún existen los módulos antiguos
print("\nVerificando módulos antiguos:")
try:
    from mesa.visualization.modules import CanvasGrid
    print("CanvasGrid está disponible")
except ImportError:
    print("No se pudo importar CanvasGrid")

try:
    from mesa.visualization.ModularVisualization import ModularServer
    print("ModularServer está disponible")
except ImportError:
    print("No se pudo importar ModularServer")