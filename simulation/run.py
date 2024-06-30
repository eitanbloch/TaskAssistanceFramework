import matplotlib
import numpy as np
from numpy import pi
from RRG import RRG
from building_blocks import Building_Blocks
from environment import Environment
from kinematics import UR5e_camera_PARAMS, UR3e_PARAMS, Transform
from visualizer import Visualize_UR

matplotlib.use('TkAgg')

if __name__ == '__main__':

    for num in range(10):
        ur_params = UR3e_PARAMS(inflation_factor=1)
        ur_params_camera = UR5e_camera_PARAMS(inflation_factor=1)
        env = Environment(env_idx=4)
        transform = Transform(ur_params)

        transform_camera = Transform(ur_params_camera)
        # moves the camera to the new position on x-axis
        transform_camera.make_camera(x_translation=1.08)

        bb = Building_Blocks(transform=transform,
                             transform_camera=transform_camera,
                             ur_params=ur_params,
                             ur_params_camera=ur_params_camera,
                             env=env,
                             resolution=0.1,
                             p_bias=0.05)

        # Visualization - Remove the following lines if you don't want to visualize the robot
        visualizer = Visualize_UR(ur_params, env=env, transform=transform, transform_camera=transform_camera, bb=bb)
        visualizer.show_conf([0, -pi / 2, 0, -pi / 2, 0, 0], [0, -pi / 2, 0, -pi / 2, 0, 0])

        rrg = RRG(bb, graph_number=num)
        rrg.run()
