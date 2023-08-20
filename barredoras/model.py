from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np
import random

class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad


class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class EstacionCarga(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

    def step(self) -> None:
        # Check if there is a robot in the same position
        # robots = self.model.grid.get_cell_list_contents([self.pos])
        # if len(robots) > 0:
        #     for r in robots:
        #         if isinstance(r, RobotLimpieza):
        #             r.recarga()
        pass
                    

class RobotLimpieza(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.estaciones_carga = []
        self.mensajes = []
        self.estado = 'limpiando' # limpiando, recargando
        self.camino = []

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    def seleccionar_nueva_pos(self, lista_de_vecinos):
        self.sig_pos = self.random.choice(lista_de_vecinos).pos

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        # #Opción 1
        # return [vecino for vecino in lista_de_vecinos
        #                 if isinstance(vecino, Celda) and vecino.sucia]
        # #Opción 2
        celdas_sucias = list()
        for vecino in lista_de_vecinos:
            if isinstance(vecino, Celda) and vecino.sucia:
                celdas_sucias.append(vecino)
        return celdas_sucias
    
    @staticmethod
    def buscar_estacion_carga(lista_de_vecinos):
        """
        Busca las estaciones de carga en la lista de vecinos y las devuelve
        en una lista.
        """
        return [vecino for vecino in lista_de_vecinos
                        if isinstance(vecino, EstacionCarga)]

    # def buscar_estacion_carga(estacionesCarga):

    def step(self):
        print(self.pos, self.carga, self.estado)
        # Obtener vecinos 
        vecinos = list(filter(
            lambda vecino: not isinstance(vecino, Mueble | RobotLimpieza),
            self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)))

        # Buscar estaciones de carga
        estaciones_carga = self.buscar_estacion_carga(vecinos)

        # Enviar mensaje a otros robots con la posición de la estación de carga
        if len(estaciones_carga) > 0:
            for estacion in estaciones_carga:
                if estacion not in self.estaciones_carga:
                    self.broadcast(('estacion', estacion.pos))
                    self.estaciones_carga.append(estacion.pos)

        # Recibir mensajes
        if len(self.mensajes) > 0:
            mensaje, pos = self.mensajes.pop()
            if mensaje == 'estacion' and pos not in self.estaciones_carga:
                self.estaciones_carga.append(pos)

        if self.estado == 'recargando':
            if len(self.camino) > 0:
                self.sig_pos = self.camino.pop()
                return
            if self.carga >= 100:
                self.estado = 'limpiando'
                self.camino = []
                self.sig_pos = self.seleccionar_nueva_pos(vecinos)
                return
            else:
                self.sig_pos = self.pos
                return

        distancia_estacion = np.inf

        caminos = {}
        camino = []
        if len(self.estaciones_carga) > 0:
            caminos = self.calcula_caminos(self.estaciones_carga)
            for c in caminos.values():
                if len(c) < distancia_estacion:
                    distancia_estacion = len(c)
                    camino = c
        
        print(distancia_estacion, len(self.estaciones_carga))
        # Va a la estacion de carga si la distancia es igual a la carga
        if len(self.estaciones_carga) and self.carga - distancia_estacion < 10:
            print('recargando')
            # Ir a estacion de carga
            self.estado = 'recargando'
            self.camino = camino
            self.sig_pos = camino.pop()
            return
        
        celdas_sucias = self.buscar_celdas_sucia(vecinos)

        if len(celdas_sucias) == 0:
            self.seleccionar_nueva_pos(vecinos)
        else:
            self.limpiar_una_celda(celdas_sucias)

    def advance(self):
        if self.pos != self.sig_pos:
            self.movimientos += 1
        
        estacion = self.model.grid.get_cell_list_contents([self.pos])
        if len(estacion) > 0:
            for e in estacion:
                if isinstance(e, EstacionCarga):
                    self.recarga()
                    break
            else:
                if self.carga > 0:
                    self.carga -= 1
            if self.sig_pos and self.sig_pos != self.pos:
                self.model.grid.move_agent(self, self.sig_pos)
            # self.model.grid.move_agent(self, self.sig_pos)

    def recarga(self):
        if self.carga >= 100:
            return
        else:
            self.carga += 25
        
    def enviar_mensaje(self, mensaje):
        self.mensajes.append(mensaje)

    def broadcast(self, mensaje):
        """
        Envía un mensaje a todos los robots.
        """
        for robot in self.model.schedule.agents:
            if isinstance(robot, RobotLimpieza):
                robot.enviar_mensaje(mensaje)

    def casilla_menor_costo(self, posiciones_no_visitadas, costos):
        costo_actual = np.inf
        pos = None

        for posicion in posiciones_no_visitadas:
            if costos[posicion] < costo_actual:
                costo_actual = costos[posicion]
                pos = posicion

        return pos

    def calcula_caminos(self, posiciones):
        """
        Calcula el camino más corto desde la posición actual hasta la posición
        indicada. Devuelve una lista de posiciones que representan el camino.
        """
        # Djisktra in python
        posiciones_no_visitadas = [pos for agent, pos in self.model.grid.coord_iter() if not isinstance(agent, Mueble)]
        posiciones2 = posiciones.copy()
        costos = {pos: np.inf for pos in posiciones_no_visitadas}
        costos[self.pos] = 0
        pos_actual = self.pos

        while len(posiciones_no_visitadas) > 0:
            # print(pos_actual)
            vecinos = self.model.grid.get_neighbors(
                pos_actual, moore=True, include_center=False)
            for vecino in vecinos:
                if isinstance(vecino, Mueble):
                    continue
                if costos[vecino.pos] > costos[pos_actual] + 1:
                    costos[vecino.pos] = costos[pos_actual] + 1
            posiciones_no_visitadas.remove(pos_actual)
            
            if pos_actual in posiciones2:
                posiciones2.remove(pos_actual)
                
                if len(posiciones2) == 0:
                    break
            pos_actual = self.casilla_menor_costo(posiciones_no_visitadas, costos)
            if pos_actual is None:
                break
        
        caminos = {}

        for pos in posiciones:
            camino = []
            pos_actual = pos
            while pos_actual != self.pos:
                camino.append(pos_actual)
                vecinos = self.model.grid.get_neighbors(
                    pos_actual, moore=True, include_center=False)
                for vecino in vecinos:
                    if costos[vecino.pos] == costos[pos_actual] - 1:
                        pos_actual = vecino.pos
                        break
            caminos[pos] = camino

        return caminos


class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 ):

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]
        self.random.shuffle(posiciones_disponibles)
        
        #                Top Left  Top Right  Bot Left  Bot Right
        estaciones_creadas = [False, False, False, False]

        for id_, pos in enumerate(posiciones_disponibles):
            if all(estaciones_creadas):
                break
            row, col = pos
            if row < M / 2 and col < N / 2 and not estaciones_creadas[0]:
                estacion_carga = EstacionCarga(int(f"{num_agentes}{id_}") + 1, self)
                self.grid.place_agent(estacion_carga, pos)
                self.schedule.add(estacion_carga)
                estaciones_creadas[0] = True
                posiciones_disponibles.remove(pos)
            elif row < M / 2 and col >= N / 2 and not estaciones_creadas[1]:
                estacion_carga = EstacionCarga(int(f"{num_agentes}{id_}") + 1, self)
                self.grid.place_agent(estacion_carga, pos)
                estaciones_creadas[1] = True
                posiciones_disponibles.remove(pos)
            elif row >= M / 2 and col < N / 2 and not estaciones_creadas[2]:
                estacion_carga = EstacionCarga(int(f"{num_agentes}{id_}") + 1, self)
                self.grid.place_agent(estacion_carga, pos)
                estaciones_creadas[2] = True
                posiciones_disponibles.remove(pos)
            elif row >= M / 2 and col >= N / 2 and not estaciones_creadas[3]:
                estacion_carga = EstacionCarga(int(f"{num_agentes}{id_}") + 1, self)
                self.grid.place_agent(estacion_carga, pos)
                estaciones_creadas[3] = True
                posiciones_disponibles.remove(pos)

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)

        for id_, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id_}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id_, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id_}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)


        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias},
        )

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents if isinstance(agent, RobotLimpieza)]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0

