"""
Registry of the available task allocation algorithms.

To add a new algorithm, implement a TaskAllocator (local) or GlobalTaskAllocator (global) subclass (see base.py) and add it to ALLOCATORS
using the name that will be passed on the command line, e.g.:

    from Simulation.TaskAllocation.my_auction import AuctionAllocator
    ALLOCATORS[AuctionAllocator.name] = AuctionAllocator
"""
from Simulation.TaskAllocation.base import TaskAllocator, GlobalTaskAllocator
from Simulation.TaskAllocation.greedy import GreedyAllocator
from Simulation.TaskAllocation.hungarian import HungarianAllocator

DEFAULT_ALLOCATOR = GreedyAllocator.name

ALLOCATORS = {
    GreedyAllocator.name: GreedyAllocator,
    HungarianAllocator.name: HungarianAllocator,
}


def get_allocator(name=None):
    """
    Returns a new instance of the task allocator registered under the given name.
    If name is None the default (greedy) allocator is returned.
    """
    if name is None:
        name = DEFAULT_ALLOCATOR
    if name not in ALLOCATORS:
        raise ValueError('Unknown task allocation algorithm "' + str(name) + '". Available: '
                         + ', '.join(sorted(ALLOCATORS.keys())))
    return ALLOCATORS[name]()
