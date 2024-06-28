import numpy as np
from numpy import pi
import time as time


def discretize_path(task_path, bb) -> (bool, list):
    # Camera configuration for the home position
    camera_home = np.array([0, -pi / 2, 0, -pi / 2, 0, 0])

    # Traversing the task path and camera configuration performing local planning
    discrete_path = np.empty((0, 6))
    for i in range(len(task_path) - 1):

        t = time.time()
        is_possible, path = bb.local_planner((task_path[i], task_path[i + 1]), (camera_home, camera_home))
        print("Time for local planner: ", i, "is", time.time() - t)

        if not is_possible:
            print("Couldn't discretize path")
            return False, []
        robot_path = path[0]
        # if discrete path is empty, add the first path
        discrete_path = np.vstack((discrete_path, robot_path))

    return True, discrete_path


def normalized_true_intervals(arr):
    intervals = []
    start = None
    N = len(arr)

    for i, (_, val) in enumerate(arr):
        if val and start is None:
            start = i
        elif not val and start is not None:
            intervals.append((start / N, (i - 1) / N))
            start = None

    if start is not None:
        intervals.append((start / N, (len(arr) - 1) / N))

    return intervals


def create_visibility_intervals(path, conf, bb):
    intervals = []
    for i in range(len(path)):
        visibility, _ = bb.is_visible(path[i], conf)
        intervals.append((path[i], visibility))

    return normalized_true_intervals(intervals)
