import os
import pickle
from time import sleep, time

from CameraController import CameraWrapper
from Robots import TaskRobot, AssistanceRobot

assistance_robot = AssistanceRobot()
task_robot = TaskRobot()


def save(filename, data):
    """
    Function to save data to a file using pickle.

    :param filename: The name of the file to save the data to.
    :param data: The data to save.
    """
    with open(filename, 'wb') as f:
        pickle.dump(data, f)


def load(filename):
    """
    Function to load data from a file using pickle.

    :param filename: The name of the file to load the data from.
    :return: The loaded data, or an empty dictionary if the file does not exist.
    """
    if not os.path.exists(filename):
        return {}
    with open(filename, 'rb') as f:
        return pickle.load(f)


def reset_robots():
    """
    Function to move the robots to their home positions.
    """
    task_robot.move_home()
    assistance_robot.move_home()


def compute_task_time():
    """
    Function to compute the average time it takes for the task robot to execute its task.
    """
    avg_time = 0
    N = 10
    for _ in range(N):
        reset_robots()
        print('Starting')
        sleep(2)
        t = time()
        task_robot.execute_task()
        while task_robot.is_running():
            pass
        avg_time += (time() - t) / N
        print(f'Task time: {round(time() - t, 3)}')

    print(f'Average: {avg_time}')
    exit(0)


def compute_intervals():
    """
    Function to compute the intervals during which an object is visible in the camera feed while the task robot is executing its task.

    :return: A list of tuples representing the start and end times of each interval.
    """
    intervals = []
    camera = CameraWrapper()
    state = camera.is_visible()
    interval_start = 0

    task_robot.execute_task()
    print('Starting...', end=' ')
    start_time = time()
    while task_robot.is_running():
        new_state = camera.is_visible(show=True)
        if new_state != state:
            if state is True:  # if stopped seeing task
                intervals.append((interval_start, time() - start_time))
            if state is False:  # if started seeing task
                interval_start = time() - start_time

            state = new_state

    total_time = time() - start_time
    print(f'Computation time: {total_time}')

    if state is True:
        intervals.append((interval_start, total_time))

    # normalize intervals
    intervals = [(start / total_time, end / total_time) for start, end in intervals]
    return intervals


def generate_intervals_from_graph(graph, configs, root=0):
    """
    Function to generate intervals for all nodes in the graph
    :param graph: adjacency list representation of the graph
    :param configs: configurations for all nodes
    :param root: root node of the graph
    """
    intervals = load('intervals.pkl')

    def aux(current_node):
        # move to the current node
        assistance_robot.move(configs[current_node])
        # compute intervals
        intervals[current_node] = compute_intervals()

        save('intervals.pkl', intervals)

        # generate intervals for all neighbors
        for v, _ in graph[current_node]:
            if v not in intervals:
                aux(v)

    aux(root)
    return intervals


def generate_intervals_from_list(samples):
    """
    Function to generate intervals for a list of samples
    :param samples: list of samples
    """
    task_robot.move_home()
    intervals = load('intervals.pkl')

    for sample in samples:
        if tuple(sample) in intervals.keys():
            print(f'skipping sample, len: {len(intervals)}')
            continue

        # go to sampled configuration
        assistance_robot.move(sample)

        # compute intervals
        intervals[tuple(sample)] = compute_intervals()
        assistance_robot.move_home()

        # save intervals
        save('intervals.pkl', intervals)

    print(intervals)


if __name__ == '__main__':
    # Usage
    samples = [assistance_robot.home]
    generate_intervals_from_list(samples)
