from typing import Tuple
from algorithms import utils
from algorithms.problems import SystemRepairProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def _distanciaManhattan(posicion1, posicion2):
    x1, y1 = posicion1
    x2, y2 = posicion2

    return abs(x1 - x2) + abs(y1 - y2)


def _distanciaEuclidiana(posicion1, posicion2):
    x1, y1 = posicion1
    x2, y2 = posicion2

    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    posicion, tieneKit, sistemasPendientes = state

    if not tieneKit:
        return _distanciaManhattan(
            posicion,
            problem.kitPosition
        )

    if len(sistemasPendientes) > 0:
        distancias = []

        for sistema in sistemasPendientes:
            distancia = _distanciaManhattan(
                posicion,
                sistema
            )
            distancias.append(distancia)

        return min(distancias)

    return _distanciaManhattan(
        posicion,
        problem.controlPosition
    )


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    posicion, tieneKit, sistemasPendientes = state

    if not tieneKit:
        return _distanciaEuclidiana(
            posicion,
            problem.kitPosition
        )

    if len(sistemasPendientes) > 0:
        distancias = []

        for sistema in sistemasPendientes:
            distancia = _distanciaEuclidiana(
                posicion,
                sistema
            )
            distancias.append(distancia)

        return min(distancias)

    return _distanciaEuclidiana(
        posicion,
        problem.controlPosition
    )


def _cotaInferiorMisionRestante(posicion,sistemasPendientes,posicionControl):
    cotaDistancia = _distanciaManhattan(posicion,posicionControl)

    for sistema in sistemasPendientes:
        distanciaPasandoPorSistema = (_distanciaManhattan(posicion, sistema)+ _distanciaManhattan(sistema, posicionControl))

        if distanciaPasandoPorSistema > cotaDistancia:
            cotaDistancia = distanciaPasandoPorSistema

    if len(sistemasPendientes) > 0:
        cotaVisitas = len(sistemasPendientes) + 1
    elif posicion == posicionControl:
        cotaVisitas = 0
    else:
        cotaVisitas = 1

    return max(cotaDistancia, cotaVisitas)

def systemRepairHeuristic(state: Tuple[Tuple, bool, Tuple], problem: SystemRepairProblem):
    """
    Your heuristic for the SystemRepairProblem.

    state: (position, hasKit, pendingSystems)
    problem: SystemRepairProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider the kit, pending systems, and the final return to control center
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    posicion, tieneKit, sistemasPendientes = state

    if problem.isGoalState(state):
        return 0

    if not tieneKit:
        distanciaHastaKit = _distanciaManhattan(posicion,problem.kitPosition)

        cacheDesdeKit = problem.heuristicInfo.setdefault("desdeKit",{})

        claveCache = tuple(sistemasPendientes)

        if claveCache not in cacheDesdeKit:
            cacheDesdeKit[claveCache] = _cotaInferiorMisionRestante(problem.kitPosition,sistemasPendientes,problem.controlPosition)

        return distanciaHastaKit + cacheDesdeKit[claveCache]

    return _cotaInferiorMisionRestante(posicion,sistemasPendientes,problem.controlPosition)
