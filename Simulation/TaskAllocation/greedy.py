"""
Greedy task allocation: the original allocation of Token Passing.
"""
import random
from Simulation.TaskAllocation.base import TaskAllocator


class GreedyAllocator(TaskAllocator):
    """
    Assigns to the agent the available task whose pickup cell is the closest one according to the
    Manhattan distance. Ties are broken by the random starting candidate, exactly as in the original
    TokenPassingRecovery.get_closest_task_name().
    """

    name = 'greedy'

    def select_task(self, agent_name, agent_pos, available_tasks):
        closest = random.choice(list(available_tasks.keys()))
        dist = self.tp.admissible_heuristic(available_tasks[closest][0], agent_pos)
        for task_name, task in available_tasks.items():
            tmp = self.tp.admissible_heuristic(task[0], agent_pos)
            if tmp < dist:
                closest = task_name
                dist = tmp
        return closest
