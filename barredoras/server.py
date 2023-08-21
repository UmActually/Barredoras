import mesa

from .model import Habitacion, RobotLimpieza, Celda, Mueble, EstacionCarga

MAX_NUMBER_ROBOTS = 20

def agent_portrayal(agent):
    if isinstance(agent, RobotLimpieza):
        return {"Shape": "circle", "Filled": "false", "Color": "Cyan", "Layer": 1, "r": 0.9,
                "text": f"{agent.carga}", "text_color": "black"}
    elif isinstance(agent, Mueble):
        return {"Shape": "rect", "Filled": "true", "Color": "black", "Layer": 0,
                "w": 0.9, "h": 0.9, "text": "🛋"}
    elif isinstance(agent, Celda):
        portrayal = {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black"}
        if agent.sucia:
            portrayal["Color"] = "#ccbeaf"
            portrayal["text"] = "💩"
        else:
            portrayal["Color"] = "white"
            portrayal["text"] = ""
        return portrayal
    elif isinstance(agent, EstacionCarga):
        return {"Shape": "rect", "Filled": "true", "Color": "green", "Layer": 0,
                "w": 0.9, "h": 0.9, "text": "⚡"}


grid = mesa.visualization.CanvasGrid(
    agent_portrayal, 20, 20, 400, 400)
chart_celdas = mesa.visualization.ChartModule(
    [{"Label": "CeldasSucias", "Color": '#36A2EB', "label": "Celdas Sucias"}],
    50, 200,
    data_collector_name="datacollector"
)
chart_movimientos = mesa.visualization.ChartModule(
    [{"Label": "Movimientos", "Color": '#FF6384', "label": "Movimientos"}],
    50, 200,
    data_collector_name="datacollector"
)
chart_tiempo_limpieza = mesa.visualization.ChartModule(
    [{"Label": "Tiempo Limpieza", "Color": '#FF6384', "label": "Tiempo Limpieza"}],
    50, 200,  # Set canvas width and height
    data_collector_name="datacollector",
)


model_params = {
    "num_agentes": mesa.visualization.Slider(
        "Número de Robots",
        5,
        2,
        MAX_NUMBER_ROBOTS,
        1,
        description="Escoge cuántos robots deseas implementar en el modelo",
    ),
    "porc_celdas_sucias": mesa.visualization.Slider(
        "Porcentaje de Celdas Sucias",
        0.3,
        0.0,
        0.75,
        0.05,
        description="Selecciona el porcentaje de celdas sucias",
    ),
    "porc_muebles": mesa.visualization.Slider(
        "Porcentaje de Muebles",
        0.1,
        0.0,
        0.25,
        0.01,
        description="Selecciona el porcentaje de muebles",
    ),
    "modo_pos_inicial": mesa.visualization.Choice(
        "Posición Inicial de los Robots",
        "Aleatoria",
        ["Fija", "Aleatoria"],
        "Seleciona la forma se posicionan los robots"
    ),
    "rows": 20,
    "cols": 20,
}

server = mesa.visualization.ModularServer(
    Habitacion, [grid, chart_celdas, chart_movimientos, chart_tiempo_limpieza],
    "botCleaner", model_params, 8521
)
