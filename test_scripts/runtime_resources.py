"""Machine-specific acceptance budgets; retain ownership tracking and RAM protection."""
import os


def memory_limits():
    """Return aggregate RSS and minimum system-available RAM in bytes."""
    values = [float(os.environ.get('HYDROMINING_RSS_GIB', '24')),
              float(os.environ.get('HYDROMINING_MIN_AVAILABLE_GIB', '8'))]
    if any(not 0 < value < 4096 for value in values):
        raise ValueError('Memory budgets must be finite positive GiB values <4096')
    return tuple(value * 1024 ** 3 for value in values)


def reserve_desktop_cpu():
    """Reserve both SMT threads of physical core zero on this workstation."""
    available = os.sched_getaffinity(0)
    workers = available - {0, 6}
    if workers:
        os.sched_setaffinity(0, workers)
