"""
Base interface for task allocation algorithms used by TokenPassingRecovery.
"""


class TaskAllocator(object):
    """
    A task allocator decides which task an idle, unassigned agent should take.

    It is called from TokenPassingRecovery.time_forward() (phase 4) once for every idle agent
    that has no task assigned yet and has at least one available task.

    To implement a new algorithm:
      1. Subclass TaskAllocator and implement select_task().
      2. Register the class in ALLOCATORS in Simulation/TaskAllocation/__init__.py.
      3. Run it with: python3 demo.py -task_allocation <name>
    """

    # Name used on the command line (-task_allocation <name>)
    name = None

    def __init__(self):
        # Set by TokenPassingRecovery when the allocator is attached to it
        self.tp = None

    def attach(self, tp):
        """
        Gives the allocator access to the TokenPassingRecovery instance, so it can read the token
        (tp.get_token()), the other agents' paths, tp.simulation.get_time(), tp.admissible_heuristic(), etc.
        """
        self.tp = tp

    def select_task(self, agent_name, agent_pos, available_tasks):
        """
        :param agent_name: name of the idle agent being handled
        :param agent_pos: [x, y] current position of the agent
        :param available_tasks: dict {task_name: [pickup_pos, delivery_pos]}, never empty
        :return: the name of the chosen task, which must be a key of available_tasks
        """
        raise NotImplementedError
