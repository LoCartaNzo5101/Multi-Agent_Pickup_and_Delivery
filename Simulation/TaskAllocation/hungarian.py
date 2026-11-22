"""
Global task allocation with the Hungarian method (Kuhn-Munkres), solved with SciPy.

scipy.optimize.linear_sum_assignment solves the same linear assignment problem as the Hungarian method and returns
an optimal assignment; internally it uses a modified Jonker-Volgenant algorithm (shortest augmenting paths), which is
faster than the textbook Hungarian method but gives the same minimum total cost.
"""
import numpy as np
from scipy.optimize import linear_sum_assignment
from Simulation.TaskAllocation.base import GlobalTaskAllocator

# Cost given to the agent-task pairs that are not allowed (the task is not available for that agent)
INFEASIBLE = float('inf')


def solve_assignment(cost):
    """
    Minimum cost assignment for any rectangular matrix (n rows, m columns) that may contain INFEASIBLE entries,
    solved with scipy.optimize.linear_sum_assignment.

    :return: list of (row, column) pairs: min(n, m) pairs at most, none of them with an INFEASIBLE cost
    """
    n = len(cost)
    m = len(cost[0]) if n > 0 else 0
    if n == 0 or m == 0:
        return []
    # linear_sum_assignment rejects matrices where a complete assignment needs an infinite entry, so INFEASIBLE is
    # replaced with a cost bigger than any feasible assignment: the solver uses those pairs only when forced to, and
    # they are dropped from the result
    matrix = np.array(cost, dtype=float)
    infeasible = np.isinf(matrix)
    finite_max = matrix[~infeasible].max() if (~infeasible).any() else 0.0
    matrix[infeasible] = finite_max * min(n, m) + 1
    rows, cols = linear_sum_assignment(matrix)
    return [(int(i), int(j)) for i, j in zip(rows, cols) if not infeasible[i, j]]


class HungarianAllocator(GlobalTaskAllocator):
    """
    At every time step assigns the idle agents to the open tasks so that the sum of the costs is minimum, with at
    most one task per agent and one agent per task. If there are more agents than tasks, some agents get no task;
    if there are more tasks than agents, the cheapest set of tasks is chosen.

    The cost of a pair is the Manhattan distance from the agent to the pickup cell of the task (the same measure used
    by the greedy allocator); override cost() to change it. Pairs where the task is not available for the agent are
    not allowed.
    """

    name = 'hungarian'

    def cost(self, agent_pos, task):
        return self.tp.admissible_heuristic(task[0], agent_pos)

    def compute_assignment(self, agents, tasks):
        agent_names = list(agents.keys())
        task_names = list(tasks.keys())
        cost = []
        for agent_name in agent_names:
            agent_pos = agents[agent_name]
            available = self.tp.get_available_tasks(agent_pos)
            cost.append([self.cost(agent_pos, tasks[t]) if t in available else INFEASIBLE for t in task_names])
        return {agent_names[i]: task_names[j] for i, j in solve_assignment(cost)}
