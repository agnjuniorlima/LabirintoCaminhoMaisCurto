
from ast import ClassDef
from math import inf

from collections import deque
from pydoc import classname
from typing import Any, List, Union, Tuple
from unittest.mock import patch

from src.problems import ProblemInterface
from src.viewer import ViewerInterface


class Node:
    # The output path is generated backwards starting from
    # the goal node, hence the need to store the parent in
    # the node.
    def __init__(self, state: Any, action=None, previous_node=None, cost=0):
        self.state = state
        self.action = action
        self.previous_node = previous_node
        self.cost = cost
        self.depth = 1 if previous_node is None else previous_node.depth + 1

    def __repr__(self):
        # return f"state={self.state}, depth={self.depth}, cost={self.cost}\n"
        return f"state={self.state}"

    """
    def __repr__(self):
        return f"Node(state={self.state}, cost={self.cost}, action={self.action}, previous_node={self.previous_node})"
    """
    def __eq__(self, n) -> bool:
        return (self.state == n.state)

    # method necessary for easily checking if nodes
    # have already been added to sets or used as keys
    # in dictionaries.
    def __hash__(self):
        return hash(self.state)

def bfs(problem: ProblemInterface, viewer: ViewerInterface) -> Tuple[List[Any], float]:
    # generated nodes that were not expanded yet
    to_explore = deque()

    # nodes whose neighbors were already generated
    expanded = set()

    # add the starting node to the list of nodes
    # yet to be expanded.
    state_node = Node(problem.initial_state())
    to_explore.append(state_node)

    # variable to store the goal node when it is found.
    goal_found = None

    # variable to count generated_nodes
    gen_nodes = 0

    # Repeat while we haven't found the goal and still have
    # nodes to expand. If there aren't further nodes
    # to expand in the search, the goal is
    # unreachable.
    while (len(to_explore) > 0) and (goal_found is None):
        # select next node or expansion
        state_node = to_explore.popleft()

        neighbors = _generate_neighbors(state_node, problem)

        for n in neighbors:
            if (n not in expanded) and (n not in to_explore):
                if problem.is_goal(n.state):
                    goal_found = n
                    break
                gen_nodes += 1
                to_explore.append(n)

        expanded.add(state_node)

        # viewer.update(state_node.state, generated=to_explore, expanded=expanded)

    path = _extract_path(goal_found)
    cost = _path_cost(problem, path)

    return path, cost, len(expanded), gen_nodes


def dfs(problem: ProblemInterface, viewer: ViewerInterface) -> Tuple[List[Any], float]:
   # generated nodes that were not expanded yet
    to_explore = deque()

    # nodes whose neighbors were already generated
    expanded = set()

    # add the starting node to the list of nodes
    # yet to be expanded.
    state_node = Node(problem.initial_state())

    to_explore.append(state_node)

    # variable to store the goal node when it is found.
    goal_found = None

    # variable to count generated_nodes
    gen_nodes = 0
    if (len(to_explore) > 0) and (goal_found is None):
        state_node = to_explore.pop()
        neighbors = _generate_neighbors(state_node, problem)
        for n in neighbors:
            if(n not in expanded) and (n not in to_explore):
                if problem.is_goal(n.state):
                    goal_found = n
                    break
                gen_nodes += 1
                to_explore.append(n)
        expanded.add(state_node)
    

    path = _extract_path(goal_found)
    cost = _path_cost(problem, path)

    return path, cost, len(expanded), gen_nodes

def dfs2(problem: ProblemInterface, viewer: ViewerInterface, depth) -> Tuple[List[Any], float]:
    """Para uso no algoritmo iterativo de busca em profundidade."""
    # generated nodes that were not expanded yet
    to_explore = deque()

    # nodes whose neighbors were already generated
    expanded = set()

    # add the starting node to the list of nodes
    # yet to be expanded.
    state_node = Node(problem.initial_state())
    to_explore.append(state_node)

    # variable to store the goal node when it is found.
    goal_found = None

    # variable to count generated_nodes
    gen_nodes = 0

    # Repeat while we haven't found the goal and still have
    # nodes to expand. If there aren't further nodes
    # to expand in the, the goal is unreachable.
    while (len(to_explore) > 0) and (goal_found is None):
        # select next node or expansion
        state_node = to_explore.pop()
        neighbors = _generate_neighbors(state_node, problem)
        for n in neighbors:
            if (n not in expanded) and (n not in to_explore):
                if problem.is_goal(n.state):
                    goal_found = n
                    break
                if state_node.depth <= depth:
                    gen_nodes += 1
                    to_explore.append(n)

        expanded.add(state_node)
                

        #viewer.update(state_node.state, generated=to_explore, expanded=expanded)

    path = _extract_path(goal_found)
    cost = _path_cost(problem, path)

    return path, cost, len(expanded), gen_nodes


def iterative_depth_first_search(problem: ProblemInterface, viewer: ViewerInterface, depth) -> Tuple[List[Any], float]:
    max_depth = depth if depth is not None else 25 # quando a profundidadde não é informada, o algoritmo assume que a profundidade máxima é 25.
    for i in range(max_depth):
        print("profundidade: ", i)
        path, cost, expanded_nodes, gen_nodes = dfs2(problem, viewer, i)
        if len(path) > 0:
            return path, cost, expanded_nodes, gen_nodes # se encontrar o caminho, retorna o caminho e o custo e interrompe o loop.
    return path, cost, expanded_nodes, gen_nodes

def uniform_cost_search(problem: ProblemInterface, viewer: ViewerInterface) -> Tuple[List[Any], float]:
    # generated nodes that were not expanded yet
    to_explore = deque()

    # nodes whose neighbors were already generated
    expanded = set()

    # add the starting node to the list of nodes
    # yet to be expanded.
    state_node = Node(problem.initial_state())
    
    to_explore.append(state_node)

    # variable to store the goal node when it is found.
    goal_found = None

    # variable to count generated_nodes
    gen_nodes = 0

    # test if the first node is the goal node
    if problem.is_goal(state_node.state):
        goal_found = state_node
        viewer.update(state_node.state,
            generated=to_explore,
            expanded=expanded)
    else:
        # Repeat while we haven't found the goal and still have
        # nodes to expand. If there aren't further nodes
        # to expand in the search, the goal is unreachable.
        while (len(to_explore) > 0) and (goal_found is None):
            to_explore = deque(sorted(to_explore, key=lambda x: x.cost)) # ordenando por custo para garantir que o menor g(n) esteja no inicio da fila
            state_node = to_explore.popleft()
            neighbors = _generate_neighbors(state_node, problem)
            for n in neighbors:
                if (n not in expanded) and (n not in to_explore):
                    if problem.is_goal(n.state):
                        goal_found = n
                        break
                    accumulated_cost(problem, state_node, n) # adicionando o custo ao novo nó e acumulando com o custo do caminho até o nó atual
                    gen_nodes += 1
                    to_explore.append(n)

            expanded.add(state_node)

            #viewer.update(state_node.state, generated=to_explore, expanded=expanded)

    path = _extract_path(goal_found)
    cost = _path_cost(problem, path)

    return path, cost, len(expanded), gen_nodes
  
def a_star_search(problem: ProblemInterface, viewer: ViewerInterface) -> Tuple[List[Any], float]:
    # generated nodes that were not expanded yet
    to_explore = deque()

    # nodes whose neighbors were already generated
    expanded = set()

    # add the starting node to the list of nodes
    # yet to be expanded.
    state_node = Node(problem.initial_state())
    
    to_explore.append(state_node)

    # variable to store the goal node when it is found.
    goal_found = None

    # variable to count generated_nodes
    gen_nodes = 0

    # test if the first node is the goal node
    if problem.is_goal(state_node.state):
        goal_found = state_node
        viewer.update(state_node.state,
            generated=to_explore,
            expanded=expanded)
    else:
        # Repeat while we haven't found the goal and still have
        # nodes to expand. If there aren't further nodes
        # to expand in the search, the goal is unreachable.
        while (len(to_explore) > 0) and (goal_found is None):
            to_explore = deque(sorted(to_explore, key=lambda x: x.cost)) # ordenando por custo para garantir que o menor g(n) esteja no inicio da fila
            state_node = to_explore.popleft()
            neighbors = _generate_neighbors(state_node, problem)
            for n in neighbors:
                if (n not in expanded) and (n not in to_explore):
                    if problem.is_goal(n.state):
                        goal_found = n
                        break
                    accumulated_cost(problem, state_node, n) # adicionando o custo ao novo nó e acumulando com o custo do caminho até o nó atual
                    cost_to_goal(problem, n) # calculando o custo do caminho até o nó objetivo
                    gen_nodes += 1
                    to_explore.append(n)

            expanded.add(state_node)

            #viewer.update(state_node.state, generated=to_explore, expanded=expanded)

    path = _extract_path(goal_found)
    cost = _path_cost(problem, path)

    return path, cost, len(expanded), gen_nodes


 

def accumulated_cost(problem, state_node, current_state_node):
        """
        Calcula o custo de sair do estado atual e ir para o estado n e soma com o custo do estado atual.
        O resultado é inserido na propriedade cost de current_state_node.
        """
        current_state_node.cost += state_node.cost + problem.step_cost(state_node.state, None, current_state_node.state)

def cost_to_goal(problem, current_state_node):
        """
        Calcula o custo de sair do estado atual e ir para o estado final e soma com o custo do estado atual.
        O resultado é inserido na propriedade cost de current_state_node.
        """
        current_state_node.cost += problem.heuristic_cost(current_state_node.state) 

def _path_cost(problem: ProblemInterface, path: List[Node]) -> float:
    if len(path) == 0:
        return inf
    cost = 0
    for i in range(1, len(path)):
        cost += problem.step_cost(path[i].previous_node.state,
                                  path[i].action,
                                  path[i].state)
    return cost

def _extract_path(goal: Union[Node, None]) -> List[Node]:
    path = []
    state_node = goal
    while state_node is not None:
        path.append(state_node)
        state_node = state_node.previous_node
    path.reverse()
    return path

def _generate_neighbors(state_node: Node, problem: ProblemInterface) -> List[Node]:
    # generate neighbors of the current state
    neighbors = []
    state = state_node.state
    available_actions = problem.actions(state)
    for action in available_actions:
        next_state = problem.transition(state, action)
        neighbors.append(Node(next_state, action, state_node))
    return neighbors

