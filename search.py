# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
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
    "*** YOUR CODE HERE ***"

    s0 = problem.getStartState()
    successors = problem.getSuccessors(s0)
    print("Initial state:", s0)
    print("Is the start a goal?", problem.isGoalState(s0))
    
    open_list = util.Stack()
    expanded_nodes = []
    explored_states = []
    
    # create source node and insert into stack
    # node = (state, accumulated_cost, action, node_id, parent_id)
    id = 0
    src_node = (s0, 0, None, id, None)
    open_list.push(src_node)

    # DFS search
    niter = 0
    while not open_list.isEmpty():
        
        print(f"open list: {open_list.list}")
        # select deepest node from the stack
        node = open_list.pop()   
        s = node[0]
        prev_cost = node[1]
        parent_id = node[3] 
        print(f"Expanding node: {node}")
        expanded_nodes.append(node)
        explored_states.append(s)
        # check if this node is a goal state
        if problem.isGoalState(s):
            print("Goal state found!")
            break

        # get successors/children
        successors = problem.getSuccessors(s)
        # create child nodes and insert them into stack
        for successor in successors:
            state = successor[0]
            # only insert if this node has not been explored before
            if state not in explored_states:
                id += 1
                action = successor[1]
                accumulated_cost = prev_cost + successor[2]
                child_node = (state, accumulated_cost, action, id, parent_id)
                open_list.push(child_node)
                print(f"Generated child node: {child_node}")

        niter += 1

    print(f"Expanded nodes: {expanded_nodes}")
    # get the action sequence 
    goal_node = expanded_nodes[-1]
    parent_id = goal_node[-1]
    actions = [goal_node[2]]
    ix = len(expanded_nodes)-1
    while ix > 0:
        ix -= 1
        node = expanded_nodes[ix]
        if (node[3] == parent_id):
            actions.insert(0,node[2])
            parent_id = node[-1]
            if(parent_id == 0):
                break

    print(f"Action sequence: {actions}")
    return actions

    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
