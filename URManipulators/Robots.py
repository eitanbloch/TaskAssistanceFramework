import threading
from time import sleep, time
import numpy as np
from numpy import pi

from urx import Robot


class BaseRobot:
    """
    A class that represents a basic robot with fundamental functionalities.
    """

    ip: int  # IP address of the robot
    home = [0, -pi / 2, 0, -pi / 2, 0, 0]  # Default home position of the robot

    def __init__(self):
        """
        Constructor of the BaseRobot class. It tries to establish a connection with the robot.
        """
        self.robot = None
        while self.robot is None:
            try:
                self.robot = Robot(self.ip, use_rt=True)
            except:
                print('Cannot connect to robot. Retrying...')
                sleep(5)

    def move(self, config):
        """
        Moves the robot to the given configuration.

        :param config: The target configuration to move the robot to.
        """
        dist = lambda a, b: np.linalg.norm(np.array(a) - np.array(b))
        try:
            self.robot.movej(config, acc=10, vel=0.5)
        except:
            pass

        while dist(self.get_config(), config) > 0.1:
            pass

    def move_home(self):
        """
        Moves the robot to its home position.
        """
        self.move(self.home)

    def get_config(self):
        """
        Returns the current configuration of the robot.

        :return: The current configuration of the robot.
        """
        return self.robot.getj()

    def execute_path(self, path, timing_profile=None):
        """
        Executes the given path with the robot.

        :param path: List of configurations that represents the path.
        :param timing_profile: Transition times between each pair of configurations.
        """
        start_time = time()
        for i, config in enumerate(path):
            self.move(config)
            time_to_sleep = 1
            if timing_profile is not None and time() - start_time < timing_profile[i]:
                time_to_sleep = max(time_to_sleep, timing_profile[i] - (time() - start_time))

            sleep(time_to_sleep)


class TaskRobot(BaseRobot):
    """
    A class representing the task robot
    """

    ip = '192.168.0.11'  # IP address of the TaskRobot

    def __init__(self):
        """
        Constructor of the TaskRobot class.
        """
        super().__init__()

        self._is_running = False  # Flag to check if the robot is currently running a task

    def execute_task(self):
        """
        Executes a predefined task with the robot.
        """

        def aux():
            task_path = [
                [0, -pi / 2, 0, -pi / 2, 0, 0],
                [
                    0.4154955744743347, -3.0889584026732386, -0.004440600983798504, -0.08255477369342046,
                    1.1764490604400635, 0.009514331817626953],
                [
                    -0.011867348347799123, -2.579003473321432, -0.001514793373644352, 0.07376722871746821,
                    1.0491279363632202, 0.5725481510162354],
                [
                    -0.014177624379293263, -1.6043607197203578, 0.033509079610006154, 0.06766728937115474,
                    -0.03832084337343389, -1.703691307698385],
                [
                    -1.1889117399798792, -0.901359037762024, -1.8020645380020142, -0.4623677295497437,
                    1.1054232120513916, -0.025052849446431935],
                [
                    -1.1465619246112269, -2.027121683160299, -1.9031667709350586, 0.6782716947742919,
                    1.0984952449798584, 0.06840498745441437],
                [
                    -1.2014954725848597, -2.2325173817076625, -1.1535048484802246, -0.04193039358172612,
                    1.1713902950286865, 0.12482447922229767],
                [
                    -1.2014835516559046, -2.2325054607787074, -1.1535197496414185, -0.0418885511210938,
                    1.1714805364608765, -2.3283541838275355],
                [
                    -1.228062931691305, -1.9083448849120082, -1.330953598022461, 0.0995076137730102, 1.2622976303100586,
                    0.0041425228118896484],
                [-1.3178303877459925, -1.0144665998271485, -1.331526756286621, -0.7953430575183411, 1.262758731842041,
                 0.00413402309641242],
                [0, -pi / 2, 0, -pi / 2, 0, 0]
            ]

            self.execute_path(task_path)
            self._is_running = False

        self._is_running = True
        t = threading.Thread(target=aux)
        t.start()

    def is_running(self):
        """
        Checks if the robot is currently running a task.

        :return: True if the robot is running a task, False otherwise.
        """
        return self._is_running


class AssistanceRobot(BaseRobot):
    """
    A class representing the assistance robot
    """

    ip = '192.168.0.10'  # IP address of the AssistanceRobot
    home = [-1.2, -pi / 2, 0, -pi / 2, 0, 0]  # Default home position of the assistance robot

    def __init__(self):
        """
        Constructor of the AssistanceRobot class.
        """
        super().__init__()
