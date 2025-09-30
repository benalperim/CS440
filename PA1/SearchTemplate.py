# Do not change the class name or add any other libraries
from queue import PriorityQueue, Queue

class Problem(object):
    def __init__(self):
        self.create_state_space()
        self.set_initial_state()
        self.create_actions()

    def create_state_space(self):
        raise NotImplementedError("Your code goes here")
    def set_initial_state(self):
        # This function should set self.initial_state
        raise NotImplementedError("Your code goes here")

    def create_actions(self):
        raise NotImplementedError("Your code goes here")
    def goal_test(self, state):
        raise NotImplementedError("Your code goes here")
    def is_goal(self, state):
        raise NotImplementedError("Your code goes here")
    def get_cost(self,action):
        raise NotImplementedError("Your code goes here")
    def heuristic(self, state, ucs_flag=False):
        if ucs_flag:
            return 0
        else:
            self.your_heuristic_function(state)
    def your_heuristic_function(self, state):
        raise NotImplementedError("Your code goes here")

    def get_successors(self, state):
        raise NotImplementedError("Your code goes here")


class Node(object):
    def __init__(self, state):
        ''' Feel free to add any additional arguments you need'''
        self.state = state
        # You should add whatever fields you need here for your search
    def get_plan(self):
        ''' Return the plan to reach self from the start state'''
        raise NotImplementedError("Your code goes here")
    def get_path_cost(self):
        ''' Return the path cost to reach self from the start state'''
        raise NotImplementedError("Your code goes here")

def astar_graph_search(problem, ucs_flag=False):
    start_state = Node(problem.initial_state)
    # Define fringe and closed list
    # It should return a Node object
    # You should pass the ucs_flag to the heuristic function
    # and you are only allowed to use the method problem.heuristic() in the search code.
    raise NotImplementedError("Your code goes here")

if __name__ == "__main__":
    ### DO NOT CHANGE THE CODE BELOW ###
    import time
    problem = Problem()
    start = time.time()
    node = astar_graph_search(problem)
    print("Time taken: ", time.time() - start)
    print("Plan: ", node.get_plan())
    print("Path Cost: ", node.get_path_cost())
    # UCS search
    start = time.time()
    node = astar_graph_search(problem, ucs_flag=True)
    print("Time taken: ", time.time() - start)
    print("Plan: ", node.get_plan())
    print("Path Cost: ", node.get_path_cost())


