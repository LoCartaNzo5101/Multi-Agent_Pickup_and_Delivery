"""
Base interfaces for the task allocation algorithms used by TokenPassingRecovery.

At every time step, phase 4 of TokenPassingRecovery.time_forward() calls the allocator in this order:

  1. allocate(agents, tasks)         once, before the agent loop (global hook), only if the allocator overrides it
  2. for every idle agent without a task, in random order:
       select_task(agent_name, agent_pos, available_tasks)
       -> path planning with A* for the returned task
       -> task_assigned(...) if both legs are found, planning_failed(...) otherwise

Local algorithms (e.g. greedy) only implement select_task(). Global algorithms (e.g. Hungarian) compute a joint
assignment in allocate() and let select_task() return the task reserved for each agent: subclass
GlobalTaskAllocator and implement compute_assignment().
"""


class TaskAllocator(object):
    """
    Base class of every task allocation algorithm.

    To implement a new algorithm:
      1. Subclass TaskAllocator (local) or GlobalTaskAllocator (global) and implement the required method.
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
        (tp.get_token()), the other agents' paths, tp.simulation.get_time(), tp.admissible_heuristic(),
        tp.get_available_tasks(agent_pos), etc.
        """
        self.tp = tp

    def has_global_step(self):
        """
        True if this allocator overrides allocate(). TokenPassingRecovery calls allocate() only in that case, so local
        allocators (e.g. greedy) skip the global hook entirely, including building its arguments.
        """
        return type(self).allocate is not TaskAllocator.allocate

    def allocate(self, agents, tasks):
        """
        Global hook, called once per time step before the agent loop, only if the subclass overrides it
        (see has_global_step()). The default does nothing.

        :param agents: dict {agent_name: [x, y]} of the idle agents that have no task assigned
        :param tasks: dict {task_name: [pickup_pos, delivery_pos]} of all the tasks not assigned yet.
                      Not every task can be taken by every agent: use tp.get_available_tasks(agent_pos)
                      to get the tasks that the agent in agent_pos is allowed to take.
        """
        pass

    def select_task(self, agent_name, agent_pos, available_tasks):
        """
        Called for every idle agent without a task, in the agent loop.

        :param agent_name: name of the idle agent being handled
        :param agent_pos: [x, y] current position of the agent
        :param available_tasks: dict {task_name: [pickup_pos, delivery_pos]}, never empty
        :return: the name of the chosen task, which must be a key of available_tasks, or None to leave the agent
                 without a task in this time step (it then idles, or moves to a free non-task endpoint if it is
                 standing on a task cell)
        """
        raise NotImplementedError

    def task_assigned(self, agent_name, task_name):
        """Called when the path for the chosen task has been found and the task has been assigned."""
        pass

    def planning_failed(self, agent_name, task_name):
        """Called when no path was found for the chosen task: the task stays unassigned in this time step."""
        pass


class GlobalTaskAllocator(TaskAllocator):
    """
    Base class of the global allocators: the joint assignment of all idle agents to the open tasks is computed
    once per time step by compute_assignment(), then the agent loop plans the paths following it.

    While agents are handled one by one, the cells taken by the newly planned paths can make the reserved task of an
    agent not available anymore. In that case, and when the path to the reserved task cannot be found, the agent falls
    back to the closest available task that is not reserved for another agent, or gets no task in this time step.
    The assignment is recomputed from scratch at the next time step.
    """

    def __init__(self):
        super(GlobalTaskAllocator, self).__init__()
        self.assignment = {}  # {agent_name: task_name} computed for the current time step

    def compute_assignment(self, agents, tasks):
        """
        :param agents: same as in allocate()
        :param tasks: same as in allocate()
        :return: dict {agent_name: task_name}; agents left out get no reserved task
        """
        raise NotImplementedError

    def allocate(self, agents, tasks):
        if len(agents) == 0 or len(tasks) == 0:
            self.assignment = {}
        else:
            self.assignment = self.compute_assignment(agents, tasks)

    def select_task(self, agent_name, agent_pos, available_tasks):
        task_name = self.assignment.get(agent_name)
        if task_name is not None and task_name in available_tasks:
            return task_name
        # Reserved task missing or not available anymore: take the closest free task, if any
        reserved = set(t for a, t in self.assignment.items() if a != agent_name)
        free_tasks = [t for t in available_tasks if t not in reserved]
        if len(free_tasks) == 0:
            return None
        return min(free_tasks, key=lambda t: self.tp.admissible_heuristic(available_tasks[t][0], agent_pos))

    def task_assigned(self, agent_name, task_name):
        self.assignment.pop(agent_name, None)

    def planning_failed(self, agent_name, task_name):
        # Release the reservation, so that the agents handled later in this time step can take the task
        self.assignment.pop(agent_name, None)
