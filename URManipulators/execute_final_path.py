from time import sleep

from Robots import TaskRobot, AssistanceRobot
from generate_intervals import reset_robots

assistance_robot = AssistanceRobot()
task_robot = TaskRobot()


def execute_final():
    """
    Function to execute the final path for the robots.
    The function first resets the robots, then starts the task for the task robot and the path for the assistance robot.
    """

    path = [[-1.2, -1.5707963267948966, 0.0, -1.5707963267948966, 0.0, 0.0],
            [-1.2678492704974573, -0.42810101926837163, -0.42377644777297974, -3.267893453637594, 0.8418788909912109,
             0.5439426898956299],
            [-1.6044653097735804, -1.5845042667784632, -0.8082376718521118, -2.3604589901366175, 0.35097357630729675,
             1.6212403774261475],
            [-1.9947007338153284, -3.281971117059225, 0.34492475191225225, -0.23017151773486333, 0.43584465980529785,
             0.009921170771121979],
            [-1.4916413466082972, -2.92838253597402, 0.9912527243243616, -0.8381026548198243, -0.25259191194643194,
             -0.34675676027406865]]
    timing_profile = [0, 4.4633755282943515, 19.11681284324827, 36.90002000324827, 43.5]
    reset_robots()
    print('Starting')
    sleep(0.5)

    task_robot.execute_task()
    assistance_robot.execute_path(path, timing_profile=timing_profile)


if __name__ == '__main__':
    execute_final()
