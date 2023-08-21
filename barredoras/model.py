from __future__ import annotations

from typing import NamedTuple, Optional, Tuple, List, Dict

from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector
import numpy as np


NO_ROBOT_OVERLAP = True


class Vecinos(NamedTuple):
    celdas_sucias: List[Celda]
    celdas_limpias: List[Celda]
    estaciones_carga: List[EstacionCarga]
    muebles: List[Mueble]

    @property
    def validos(self) -> List[Celda]:
        return self.celdas_sucias + self.celdas_limpias + self.estaciones_carga


class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False) -> None:
        super().__init__(unique_id, model)
        self.sucia = suciedad


class Mueble(Agent):
    def __init__(self, unique_id, model) -> None:
        super().__init__(unique_id, model)


class EstacionCarga(Agent):
    def __init__(self, unique_id, model) -> None:
        super().__init__(unique_id, model)

    def step(self) -> None:
        pass


class RobotLimpieza(Agent):
    def __init__(self, unique_id: int, model: Habitacion) -> None:
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.estaciones_carga = []
        self.mensajes = []
        self.estado = 'limpiando'  # limpiando, recargando
        self.camino = []
        self.celdas_visitadas = [False for _ in range(self.model.grid.width * self.model.grid.height)]

    def step(self) -> None:
        if all(self.celdas_visitadas):
            self.model.running = False
            return

        # Buscar celdas válidas
        vecinos = self.analizar_vecinos()

        # Enviar mensaje a otros robots con la posición de la estación de carga
        for estacion in vecinos.estaciones_carga:
            if estacion not in self.estaciones_carga:
                self.broadcast('estacion_descubierta', estacion.pos)
                self.estaciones_carga.append(estacion.pos)

        for mueble in vecinos.muebles:
            self.broadcast('marcar_celda', mueble.pos)

        # Recibir mensajes
        if self.mensajes:
            for mensaje, contenido in self.mensajes:
                if mensaje == 'estacion_descubierta' and contenido not in self.estaciones_carga:
                    self.estaciones_carga.append(contenido)
                elif mensaje == 'marcar_celda':
                    self.celdas_visitadas[contenido[0] * self.model.grid.width + contenido[1]] = True
            self.mensajes.clear()

        if self.estado == 'recargando':
            if self.camino:
                self.sig_pos = self.camino[-1]
                return
            if self.carga >= 100:
                self.estado = 'limpiando'
                self.camino.clear()
                self.seleccionar_nueva_pos(vecinos.validos)
                return
            else:
                self.sig_pos = self.pos
                return

        # Camino más cercano a una estación de carga
        if self.estaciones_carga:
            camino = min(
                self.calcula_caminos(self.estaciones_carga).values(),
                key=len)
        else:
            camino = []

        # Va a la estacion de carga si la distancia es igual a la carga
        if self.estaciones_carga and self.carga - len(camino) < 10:
            # Ir a estacion de carga
            self.estado = 'recargando'
            self.camino = camino
            self.sig_pos = camino[-1]
            return

        if vecinos.celdas_sucias:
            self.limpiar_una_celda(vecinos.celdas_sucias)
        else:
            self.seleccionar_nueva_pos(vecinos.validos)

    def advance(self) -> None:
        tener_piedad = False

        if NO_ROBOT_OVERLAP:
            for agent in self.model.schedule.agents:
                if (isinstance(agent, RobotLimpieza) and agent != self
                        and agent.sig_pos == self.sig_pos
                        and agent.sig_pos != self.pos
                        and agent.unique_id > self.unique_id):
                    # Que no se le acabe la pila esperando a otro robot que no se mueve
                    tener_piedad = bool(self.camino)

        agentes_aqui = self.model.grid.get_cell_list_contents([self.pos])
        if agentes_aqui and any(isinstance(agente, EstacionCarga) for agente in agentes_aqui):
            self.recarga()
        elif self.carga > 0 and not tener_piedad:
            self.carga -= 1
        if self.camino and not tener_piedad:
            self.camino.pop()
        if self.sig_pos and self.sig_pos != self.pos:
            self.movimientos += 1
            self.model.grid.move_agent(self, self.sig_pos)
            self.broadcast('marcar_celda', self.pos)

    def analizar_vecinos(self) -> Vecinos:
        vecinos = {
            'celdas_sucias': [],
            'celdas_limpias': [],
            'estaciones_carga': [],
            'muebles': []
        }

        for celda in self.model.grid.get_neighbors(
                self.pos, moore=True, include_center=False):
            if isinstance(celda, RobotLimpieza):
                continue
            if isinstance(celda, Celda):
                if celda.sucia:
                    vecinos['celdas_sucias'].append(celda)
                else:
                    vecinos['celdas_limpias'].append(celda)
            elif isinstance(celda, EstacionCarga):
                vecinos['estaciones_carga'].append(celda)
            elif isinstance(celda, Mueble):
                vecinos['muebles'].append(celda)

        return Vecinos(**vecinos)

    def celda_ya_visitada(self, pos: Tuple[int, int]) -> bool:
        try:
            return self.celdas_visitadas[pos[0] * self.model.grid.width + pos[1]]
        except IndexError:
            # No sé por qué, pero ya me falló una vez
            return False

    def limpiar_una_celda(self, celdas) -> None:
        celda_a_limpiar = self.random.choice(celdas)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    def seleccionar_nueva_pos(self, celdas) -> None:
        celdas_no_visitadas = [
            celda for celda in celdas
            if not self.celda_ya_visitada(celda.pos)]

        if celdas_no_visitadas:
            self.sig_pos = self.random.choice(celdas_no_visitadas).pos
        else:
            self.sig_pos = self.random.choice(celdas).pos

    def recarga(self) -> None:
        if self.carga >= 100:
            return
        self.carga += min(100 - self.carga, 25)

    def enviar_mensaje(self, mensaje, contenido) -> None:
        self.mensajes.append((mensaje, contenido))

    def broadcast(self, mensaje, contenido) -> None:
        """
        Envía un mensaje a todos los robots.
        """
        for robot in self.model.schedule.agents:
            if isinstance(robot, RobotLimpieza):
                robot.enviar_mensaje(mensaje, contenido)

    @staticmethod
    def casilla_menor_costo(posiciones_no_visitadas, costos) -> Optional[Tuple[int, int]]:
        if not posiciones_no_visitadas:
            return None
        return min(posiciones_no_visitadas, key=lambda pos: costos[pos])

        # costo_actual = np.inf
        # pos = None
        #
        # for posicion in posiciones_no_visitadas:
        #     if costos[posicion] < costo_actual:
        #         costo_actual = costos[posicion]
        #         pos = posicion
        #
        # return pos

    def calcula_caminos(self, posiciones) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
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
    def __init__(
            self, rows: int, cols: int, num_agentes: int = 5,
            porc_celdas_sucias: float = 0.6, porc_muebles: float = 0.1,
            modo_pos_inicial: str = 'Fija') -> None:
        super().__init__()

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(rows, cols, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]
        self.random.shuffle(posiciones_disponibles)

        #                Top Left  Top Right  Bot Left  Bot Right
        estaciones_creadas = [False, False, False, False]

        for id_, pos in enumerate(posiciones_disponibles):
            # Esto es para que PyCharm no llore
            pos: Tuple[int, int] = pos
            if all(estaciones_creadas):
                break
            row, col = pos
            if row < rows / 2 and col < cols / 2 and not estaciones_creadas[0]:
                estacion_carga = EstacionCarga(int(f"{num_agentes}{id_}") + 1, self)
                self.grid.place_agent(estacion_carga, pos)
                self.schedule.add(estacion_carga)
                estaciones_creadas[0] = True
                posiciones_disponibles.remove(pos)
            elif row < rows / 2 and col >= cols / 2 and not estaciones_creadas[1]:
                estacion_carga = EstacionCarga(int(f"{num_agentes}{id_}") + 1, self)
                self.grid.place_agent(estacion_carga, pos)
                estaciones_creadas[1] = True
                posiciones_disponibles.remove(pos)
            elif row >= rows / 2 and col < cols / 2 and not estaciones_creadas[2]:
                estacion_carga = EstacionCarga(int(f"{num_agentes}{id_}") + 1, self)
                self.grid.place_agent(estacion_carga, pos)
                estaciones_creadas[2] = True
                posiciones_disponibles.remove(pos)
            elif row >= rows / 2 and col >= cols / 2 and not estaciones_creadas[3]:
                estacion_carga = EstacionCarga(int(f"{num_agentes}{id_}") + 1, self)
                self.grid.place_agent(estacion_carga, pos)
                estaciones_creadas[3] = True
                posiciones_disponibles.remove(pos)

        # Posicionamiento de muebles
        num_muebles = int(rows * cols * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)

        for id_, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id_}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(rows * cols * porc_celdas_sucias)
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

        for id_ in range(num_agentes):
            robot = RobotLimpieza(id_, self)
            self.grid.place_agent(robot, pos_inicial_robots[id_])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias},
        )

    def step(self) -> None:
        self.datacollector.collect(self)
        self.schedule.step()

    def todo_limpio(self) -> bool:
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


def get_cargas(model: Model) -> list[tuple[int, int]]:
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


def get_movimientos(agent: Agent) -> Dict[int, int]:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0
