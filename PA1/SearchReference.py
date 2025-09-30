"""Minimal reference implementation of A* / UCS for the CS440 PA1 assignment.

This file provides a self-contained ExampleProblem, Node, and astar_graph_search
implementation you can use as a working reference. It intentionally mirrors the
public API used in `SearchTemplate.py` but lives in a separate file so the
original template remains untouched.
"""
from queue import PriorityQueue


class ExampleProblem:
    """Small example graph problem.

    States: 'A' start, 'G' goal. Graph edges have non-negative costs.
    Successor tuples: (action, next_state, cost)
    """
    def __init__(self):
        self.create_state_space()
        self.set_initial_state()
        self.create_actions()

    def create_state_space(self):
        # adjacency: state -> list of (action_name, neighbor, cost)
        self.adj = {
            'A': [('A->B','B',1), ('A->C','C',4)],
            'B': [('B->C','C',2), ('B->D','D',5)],
            'C': [('C->G','G',3)],
            'D': [('D->G','G',2)],
            'G': []
        }

    def set_initial_state(self):
        self.initial_state = 'A'

    def create_actions(self):
        # actions are implicitly encoded in adjacency lists above
        pass

    def goal_test(self, state):
        return state == 'G'

    def is_goal(self, state):
        return self.goal_test(state)

    def get_cost(self, action):
        # action is a tuple name here; cost is determined by successor tuples
        # Not used directly in this example.
        return 0

    def heuristic(self, state, ucs_flag=False):
        # If UCS is requested, return 0 so algorithm behaves like Uniform Cost Search
        if ucs_flag:
            return 0
        return self.your_heuristic_function(state)

    def your_heuristic_function(self, state):
        # Simple admissible heuristic (straight-line guess) for this tiny graph.
        h = {'A': 3, 'B': 2, 'C': 1, 'D': 1, 'G': 0}
        return h.get(state, 0)

    def get_successors(self, state):
        # Return list of (action_name, next_state, cost)
        return self.adj.get(state, [])


class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def get_plan(self):
        # Return sequence of actions from root to this node
        actions = []
        node = self
        while node and node.action is not None:
            actions.append(node.action)
            node = node.parent
        return list(reversed(actions))

    def get_path_cost(self):
        return self.path_cost


def astar_graph_search(problem, ucs_flag=False):
    """A simple A* graph-search implementation.

    Fringe entries are (f, g, counter, node) to ensure deterministic ordering.
    """
    counter = 0
    start = Node(problem.initial_state, parent=None, action=None, path_cost=0)
    start_f = start.path_cost + problem.heuristic(start.state, ucs_flag)

    fringe = PriorityQueue()
    fringe.put((start_f, start.path_cost, counter, start))
    counter += 1

    # best_g stores the best (lowest) known g for each explored state
    best_g = {start.state: 0}

    while not fringe.empty():
        f, g, _, node = fringe.get()
        # Goal test
        if problem.is_goal(node.state):
            return node

        # Expand
        for action, succ, cost in problem.get_successors(node.state):
            new_g = node.path_cost + cost
            # If we have seen a better path before, skip
            if succ in best_g and new_g >= best_g[succ]:
                continue

            child = Node(succ, parent=node, action=action, path_cost=new_g)
            h = problem.heuristic(succ, ucs_flag)
            new_f = new_g + h
            best_g[succ] = new_g
            fringe.put((new_f, new_g, counter, child))
            counter += 1

    # failure
    return None


if __name__ == "__main__":
    import time
    problem = ExampleProblem()

    start = time.time()
    node = astar_graph_search(problem)
    print("A* Time taken:", time.time() - start)
    if node:
        print("A* Plan:", node.get_plan())
        print("A* Path Cost:", node.get_path_cost())
    else:
        print("A* failed to find a solution")

    # UCS (heuristic disabled via ucs_flag)
    start = time.time()
    node = astar_graph_search(problem, ucs_flag=True)
    print("UCS Time taken:", time.time() - start)
    if node:
        print("UCS Plan:", node.get_plan())
        print("UCS Path Cost:", node.get_path_cost())
    else:
        print("UCS failed to find a solution")
