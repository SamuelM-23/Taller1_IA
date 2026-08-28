from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyDiagnosticSearch(problem: SearchProblem):
    """
    Returns a hard-coded sequence of moves for the tinyDiagnostic layout.
    For any other station layout, the sequence of moves will be incorrect.
    """
    s = Directions.SOUTH
    e = Directions.EAST
    return [s, e, s, e, e, e, e, s, e, e, s, s, e, s, s, e, s, e, e, e, e, e, e, e]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    inicial = problem.getStartState()

    pila = utils.Stack()

    pila.push((inicial, []))
    visitados = [inicial]

    while not pila.isEmpty():
        estado, camino = pila.pop()
        if problem.isGoalState(estado):
            return camino
            
        sucesores = problem.getSuccessors(estado)
        for sucesor, accion, costo in sucesores:
            if sucesor not in visitados:
                visitados.append(sucesor)
                pila.push((sucesor, camino + [accion]))

    return []


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    inicial = problem.getStartState()

    cola = utils.Queue()

    cola.push((inicial, []))
    visitados = [inicial]

    while not cola.isEmpty():
        estado, camino = cola.pop()
       
        if problem.isGoalState(estado):
            return camino
                        
        sucesores = problem.getSuccessors(estado)
        for sucesor, accion, costo in sucesores:
            if sucesor not in visitados:
                visitados.append(sucesor)
                cola.push((sucesor, camino + [accion]))

    return []


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """

    inicial = problem.getStartState()
    
    frontera = utils.PriorityQueue()
    
    frontera.push((inicial, [], 0), 0)
    
    alcanzados  = {inicial : 0}
    expandidos = []
    
    while not frontera.isEmpty():
        estado, camino, costo = frontera.pop()
        if problem.isGoalState(estado):
            return camino
        if estado not in expandidos:
            expandidos.append(estado)
            sucesores = problem.getSuccessors(estado)
            for sucesor, accion, costo_accion in sucesores:
                nuevo_costo= costo_accion+costo
                nuevo_camino= camino + [accion]
                if sucesor not in alcanzados  or nuevo_costo < alcanzados[sucesor]:
                    alcanzados[sucesor]= nuevo_costo
                    frontera.push((sucesor, nuevo_camino, nuevo_costo), nuevo_costo)
    return []
            

                
        
    
    


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    
    
    inicial = problem.getStartState()

    frontera = utils.PriorityQueue()
    frontera.push((inicial, [], 0), heuristic(inicial, problem))

    mejor_costo = {inicial: 0}

    while not frontera.isEmpty():
        estado, camino, costo = frontera.pop()

        if costo == mejor_costo.get(estado):
            if problem.isGoalState(estado):
                return camino

            for sucesor, accion, costo_accion in problem.getSuccessors(estado):
                nuevo_costo = costo + costo_accion

                if sucesor not in mejor_costo or nuevo_costo < mejor_costo[sucesor]:
                    mejor_costo[sucesor] = nuevo_costo
                    nuevo_camino = camino + [accion]
                    prioridad = nuevo_costo + heuristic(sucesor, problem)
                    frontera.push((sucesor, nuevo_camino, nuevo_costo), prioridad)

    return []
# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
