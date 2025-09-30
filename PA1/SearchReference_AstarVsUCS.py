"""Reference example showing A* and UCS produce different plans (same cost).

Graph layout (costs):
  A -> C (2)
  A -> B (2)
  B -> G (4)
  C -> G (4)

Both A-B-G and A-C-G have total cost 6. Successor order from A lists C first,
so when heuristic is disabled (UCS) the algorithm will expand C before B and
may return A->C->G. With the provided heuristic (which prefers B) A* expands
B first and returns A->B->G. This demonstrates how heuristics influence search
ordering even when optimal costs are equal.
"""
from queue import PriorityQueue


class ExampleProblemDiff:
    def __init__(self):
        self.create_state_space()
        self.set_initial_state()
        self.create_actions()

    def create_state_space(self):
        # Note: order matters here. We intentionally put C before B so UCS (by FIFO tie)
        # may prefer C when g-values tie.
        self.adj = {
            'A': [('A->C','C',2), ('A->B','B',2)],
            'B': [('B->G','G',4)],
            'C': [('C->G','G',4)],
            'G': []
        }

    def set_initial_state(self):
        self.initial_state = 'A'

    def create_actions(self):
        pass

    def goal_test(self, state):
        return state == 'G'

    def is_goal(self, state):
        return self.goal_test(state)

    def get_cost(self, action):
        return 0

    def heuristic(self, state, ucs_flag=False):
        if ucs_flag:
            return 0
        return self.your_heuristic_function(state)

    def your_heuristic_function(self, state):
        # Heuristic prefers B path: make h(B) smaller than h(C) while remaining admissible
        h = {'A': 3, 'B': 1, 'C': 3, 'G': 0}
        return h.get(state, 0)

    def get_successors(self, state):
        return self.adj.get(state, [])


class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def get_plan(self):
        actions = []
        node = self
        while node and node.action is not None:
            actions.append(node.action)
            node = node.parent
        return list(reversed(actions))

    def get_path_cost(self):
        return self.path_cost


def astar_graph_search(problem, ucs_flag=False):
    counter = 0
    start = Node(problem.initial_state, parent=None, action=None, path_cost=0)
    start_f = start.path_cost + problem.heuristic(start.state, ucs_flag)

    fringe = PriorityQueue()
    fringe.put((start_f, start.path_cost, counter, start))
    counter += 1

    best_g = {start.state: 0}

    while not fringe.empty():
        f, g, _, node = fringe.get()
        if problem.is_goal(node.state):
            return node

        for action, succ, cost in problem.get_successors(node.state):
            new_g = node.path_cost + cost
            if succ in best_g and new_g >= best_g[succ]:
                continue

            child = Node(succ, parent=node, action=action, path_cost=new_g)
            h = problem.heuristic(succ, ucs_flag)
            new_f = new_g + h
            best_g[succ] = new_g
            fringe.put((new_f, new_g, counter, child))
            counter += 1

    return None


if __name__ == "__main__":
    import time
    problem = ExampleProblemDiff()

    start = time.time()
    node = astar_graph_search(problem)
    print("A* Time taken:", time.time() - start)
    if node:
        print("A* Plan:", node.get_plan())
        print("A* Path Cost:", node.get_path_cost())

    start = time.time()
    node = astar_graph_search(problem, ucs_flag=True)
    print("UCS Time taken:", time.time() - start)
    if node:
        print("UCS Plan:", node.get_plan())
        print("UCS Path Cost:", node.get_path_cost())
