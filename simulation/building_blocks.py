import numpy as np
from numpy import pi

import intervals_runner

# camera parameters
fov_horizontal_radius = 70.74957348407156
fov_vertical_radius = 43.54111545632642


class Building_Blocks(object):
    """
    @param resolution determines the resolution of the local planner(how many intermediate configurations to check)
    @param p_bias determines the probability of the sample function to return the goal configuration
    """

    def __init__(self, transform, transform_camera, ur_params, ur_params_camera, env, resolution=0.1, p_bias=0.05):
        self.transform = transform
        self.transform_camera = transform_camera
        self.ur_params = ur_params
        self.ur_params_camera = ur_params_camera
        self.env = env
        self.resolution = resolution
        self.p_bias = p_bias
        self.cost_weights = np.array([0.4, 0.3, 0.2, 0.1, 0.07, 0.05])
        self.wall_constraint = True

        task_path = [
            [0, -pi / 2, 0, -pi / 2, 0, 0],
            [
                0.4154955744743347, -3.0889584026732386, -0.004440600983798504, -0.08255477369342046,
                1.1764490604400635,
                0.009514331817626953],
            [
                -0.011867348347799123, -2.579003473321432, -0.001514793373644352, 0.07376722871746821,
                1.0491279363632202,
                0.5725481510162354],
            [
                -0.014177624379293263, -1.6043607197203578, 0.033509079610006154, 0.06766728937115474,
                -0.03832084337343389,
                -1.703691307698385],
            [
                -1.1889117399798792, -0.901359037762024, -1.8020645380020142, -0.4623677295497437, 1.1054232120513916,
                -0.025052849446431935],
            [
                -1.1465619246112269, -2.027121683160299, -1.9031667709350586, 0.6782716947742919, 1.0984952449798584,
                0.06840498745441437],
            [
                -1.2014954725848597, -2.2325173817076625, -1.1535048484802246, -0.04193039358172612, 1.1713902950286865,
                0.12482447922229767],
            [
                -1.2014835516559046, -2.2325054607787074, -1.1535197496414185, -0.0418885511210938, 1.1714805364608765,
                -2.3283541838275355],
            [
                -1.228062931691305, -1.9083448849120082, -1.330953598022461, 0.0995076137730102, 1.2622976303100586,
                0.0041425228118896484],
            [-1.3178303877459925, -1.0144665998271485, -1.331526756286621, -0.7953430575183411, 1.262758731842041,
             0.00413402309641242],
            [0, -pi / 2, 0, -pi / 2, 0, 0]
        ]
        self.intervals_path = intervals_runner.discretize_path(np.array(task_path), self)[1]
        print(len(self.intervals_path))

    def sample(self, goal_conf) -> np.array:
        """
        sample random configuration
        @param goal_conf - the goal configuration
        """
        if np.random.random() < self.p_bias:
            return goal_conf
        else:
            sample = []
            for min_val, max_val in self.ur_params.mechanical_limits.values():
                sample.append(np.random.uniform(min_val, max_val))

            return np.array(sample)

    @staticmethod
    def are_coords_colliding(coords1, coords2, max_dist) -> bool:
        for sphere1 in coords1:
            for sphere2 in coords2:
                dist = np.linalg.norm(sphere1 - sphere2, ord=2)
                if dist < max_dist:
                    return True

    def robot_obstacle_collision(self, global_sphere_coords) -> bool:
        for i in range(len(self.ur_params.ur_links) - 2):
            for j in range(i + 2, len(self.ur_params.ur_links)):
                link1 = self.ur_params.ur_links[i]
                link2 = self.ur_params.ur_links[j]
                radiuses_sum = self.ur_params.sphere_radius[link1] + self.ur_params.sphere_radius[link2]

                sphere_coords1 = np.array(global_sphere_coords[link1])
                sphere_coords2 = np.array(global_sphere_coords[link2])
                sub_rows = (sphere_coords1[:, np.newaxis, :] - sphere_coords2[np.newaxis, :, :]).astype(np.float64)
                dists = np.linalg.norm(sub_rows, axis=2)
                if np.any(dists < radiuses_sum):
                    print("robot robot collision")
                    return True

        for link in self.ur_params.ur_links:
            radiuses_sum = self.ur_params.sphere_radius[link] + self.env.radius
            for x1_coord, y1_coord, z1_coord, rad1 in global_sphere_coords[link]:
                if z1_coord < 0:
                    return True

                x1_coord = x1_coord
                if type(x1_coord) is np.ndarray:
                    x1_coord = x1_coord[0]
                y1_coord = y1_coord
                if type(y1_coord) is np.ndarray:
                    y1_coord = y1_coord[0]
                z1_coord = z1_coord
                if type(z1_coord) is np.ndarray:
                    z1_coord = z1_coord[0]

                coords1 = np.array([x1_coord, y1_coord, z1_coord])
                if self.env.obstacles.size != 0:
                    dists = np.linalg.norm(self.env.obstacles - coords1, axis=1)
                    if np.any(dists < radiuses_sum):
                        return True
        return False

    def camera_obstacle_collision(self, global_sphere_coords) -> bool:
        for i in range(len(self.ur_params_camera.ur_links) - 2):
            for j in range(i + 2, len(self.ur_params_camera.ur_links)):
                link1 = self.ur_params_camera.ur_links[i]
                link2 = self.ur_params_camera.ur_links[j]
                radiuses_sum = self.ur_params_camera.sphere_radius[link1] + self.ur_params_camera.sphere_radius[link2]

                sphere_coords1 = np.array(global_sphere_coords[link1])
                sphere_coords2 = np.array(global_sphere_coords[link2])
                sub_rows = (sphere_coords1[:, np.newaxis, :] - sphere_coords2[np.newaxis, :, :]).astype(np.float64)
                dists = np.linalg.norm(sub_rows, axis=2)
                if np.any(dists < radiuses_sum):
                    return True

        for link in self.ur_params_camera.ur_links:
            radiuses_sum = self.ur_params_camera.sphere_radius[link] + self.env.radius
            for x1_coord, y1_coord, z1_coord, rad1 in global_sphere_coords[link]:
                if z1_coord < 0:
                    return True

                x1_coord = x1_coord
                if type(x1_coord) is np.ndarray:
                    x1_coord = x1_coord[0]
                y1_coord = y1_coord
                if type(y1_coord) is np.ndarray:
                    y1_coord = y1_coord[0]
                z1_coord = z1_coord
                if type(z1_coord) is np.ndarray:
                    z1_coord = z1_coord[0]

                coords1 = np.array([x1_coord, y1_coord, z1_coord])
                if self.env.obstacles.size != 0:
                    dists = np.linalg.norm(self.env.obstacles - coords1, axis=1)
                    if np.any(dists < radiuses_sum):
                        return True

        return False

    def camera_robot_collision(self, global_sphere_coords, global_sphere_coords_camera) -> bool:
        for link in self.ur_params.ur_links:
            for x1_coord, y1_coord, z1_coord, rad1 in global_sphere_coords[link]:
                if z1_coord < 0:
                    return True
                x1_coord = x1_coord
                if type(x1_coord) is np.ndarray:
                    x1_coord = x1_coord[0]
                y1_coord = y1_coord
                if type(y1_coord) is np.ndarray:
                    y1_coord = y1_coord[0]
                z1_coord = z1_coord
                if type(z1_coord) is np.ndarray:
                    z1_coord = z1_coord[0]

                coords1 = np.array([x1_coord, y1_coord, z1_coord])

                for link in self.ur_params_camera.ur_links:
                    radiuses_sum = self.ur_params_camera.sphere_radius[link] + self.ur_params.sphere_radius[link]
                    for x1_cam_coord, y1_cam_coord, z1_cam_coord, rad1_cam in global_sphere_coords_camera[link]:
                        if z1_cam_coord < 0:
                            return True

                        x1_cam_coord = x1_cam_coord
                        if type(x1_cam_coord) is np.ndarray:
                            x1_cam_coord = x1_cam_coord[0]
                        y1_cam_coord = y1_cam_coord
                        if type(y1_cam_coord) is np.ndarray:
                            y1_cam_coord = y1_cam_coord[0]
                        z1_cam_coord = z1_cam_coord
                        if type(z1_cam_coord) is np.ndarray:
                            z1_cam_coord = z1_cam_coord[0]

                        coords_cam = np.array([x1_cam_coord, y1_cam_coord, z1_cam_coord])

                        dists = np.linalg.norm(coords1 - coords_cam, ord=2)
                        if np.any(dists < radiuses_sum):
                            return True
        return False

    def is_in_collision(self, conf, conf_camera) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration
        """

        global_sphere_coords = self.transform.conf2sphere_coords(conf)

        global_sphere_coords_camera = self.transform_camera.conf2sphere_coords(conf_camera)

        robot_camera_collision = self.camera_robot_collision(global_sphere_coords, global_sphere_coords_camera)

        if robot_camera_collision:
            return True

        return False

    def calc_angle_between_vecs(self, v1, v2):
        """Calculate the angle between two vectors in radians."""
        dot_product = np.dot(v1, v2)
        norm_product = np.linalg.norm(v1) * np.linalg.norm(v2)
        cos_angle = dot_product / norm_product
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        return angle

    def compute_fov_angles(self, dx, dy, dz):
        """Compute horizontal and vertical angles of the direction vector."""
        # Calculate the magnitude of the vector
        magnitude = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        # Normalize the vector
        dx_norm = dx / magnitude
        dy_norm = dy / magnitude
        dz_norm = dz / magnitude

        # Compute horizontal angle (in radians)
        theta_horizontal = np.arctan2(dy_norm, dx_norm)

        # Compute vertical angle (in radians)
        theta_vertical = np.arctan2(dz_norm, dx_norm)

        return theta_horizontal, theta_vertical

    def is_legal_angle(self, global_sphere_coords, global_sphere_coords_camera) -> bool:

        # get basic line between coord -1 and -2
        num_of_spheres = len(global_sphere_coords_camera["wrist_3_link"])
        x_coord_cam1, y_coord_cam1, z_coord_cam1, rad_cam1 = global_sphere_coords_camera["wrist_3_link"][
            num_of_spheres - 1]
        x_coord_cam2, y_coord_cam2, z_coord_cam2, rad_cam2 = global_sphere_coords_camera["wrist_3_link"][
            num_of_spheres - 2]

        x_coord, y_coord, z_coord, rad = global_sphere_coords["wrist_3_link"][num_of_spheres - 1]

        # calculate  the two points
        p1 = (x_coord_cam1, y_coord_cam1, z_coord_cam1)
        p2 = (x_coord_cam2, y_coord_cam2, z_coord_cam2)
        task_p = (x_coord, y_coord, z_coord)
        task_p = np.array(task_p)

        # calculate the vector between the two points
        p1 = np.array(p1)
        p2 = np.array(p2)
        # from p2 to p1
        cam_vec = p1 - p2

        # from cam_vec to (0,0,0) angles
        cam_vec = cam_vec / np.linalg.norm(cam_vec)
        cam_angle_x, cam_angle_y, cam_angle_z = np.arccos(cam_vec[0]), np.arccos(cam_vec[1]), np.arccos(cam_vec[2])

        # calculate the vector between the camera and the task point
        end_cam_to_task = task_p - p1
        end_cam_to_task = end_cam_to_task / np.linalg.norm(end_cam_to_task)
        end_cam_to_task_angle_x, cam_to_task_angle_y, cam_to_task_angle_z = np.arccos(end_cam_to_task[0]), np.arccos(
            end_cam_to_task[1]), np.arccos(end_cam_to_task[2])

        fin_angle_x, fin_angle_y = abs(np.rad2deg(cam_angle_x - end_cam_to_task_angle_x)), abs(
            np.rad2deg(cam_angle_y - cam_to_task_angle_y))

        if fin_angle_x > fov_horizontal_radius or fin_angle_y > fov_vertical_radius:
            return False
        else:
            return True

    def is_visible(self, conf, conf_camera) -> (bool, list):

        # get camera position
        global_sphere_coords_camera = self.transform_camera.conf2sphere_coords(conf_camera)
        num_of_spheres = len(global_sphere_coords_camera["wrist_3_link"])
        x1_coord_cam, y1_coord_cam, z1_coord_cam, rad1_cam = global_sphere_coords_camera["wrist_3_link"][
            num_of_spheres - 1]

        # get end effector position
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        num_of_spheres = len(global_sphere_coords["wrist_3_link"])
        x1_coord, y1_coord, z1_coord, rad1 = global_sphere_coords["wrist_3_link"][num_of_spheres - 1]

        # check angle is legal
        if not self.is_legal_angle(global_sphere_coords, global_sphere_coords_camera):
            # print("False - Illegal angle")
            return False, []

        conf_camera = np.array([x1_coord_cam, y1_coord_cam, z1_coord_cam])
        conf = np.array([x1_coord, y1_coord, z1_coord])

        # checking no collision with obstacles
        min_confs_num = 3
        max_dist_cam = np.max(np.abs(conf - conf_camera))
        confs_num = int(max_dist_cam / self.resolution)
        confs_num = max(confs_num, min_confs_num)
        confs = np.linspace(start=conf_camera, stop=conf, num=confs_num, endpoint=True)

        # make sure that doesn't intersect with obstacles or robot parts
        confs_w_radius = [np.append(arr, 1) for arr in confs]

        for c in confs:
            radiuses_sum = self.ur_params.sphere_radius["wrist_3_link"] + self.env.radius
            dists = np.linalg.norm(self.env.obstacles - c, axis=1)
            if np.any(dists < radiuses_sum):
                # print("False - Obstacles on the way")
                return False, confs_w_radius

        return True, confs_w_radius

    def local_planner(self, robot_confs: tuple, camera_confs: tuple) -> (bool, list):
        """
        check for collisions between two configurations - return True if transition is valid
        plan for each one and someone is finishing before the other keep him in the last configuration
        @param robot_confs - tuple of (start configuration, end configuration)
        @param camera_confs - tuple of (start configuration, end configuration)
        """

        # plan the motion for the robot
        current_conf = robot_confs[0]
        prev_conf = robot_confs[1]
        current_conf_camera = camera_confs[0]
        prev_conf_camera = camera_confs[1]

        confs_camera = np.array([prev_conf_camera])
        confs = np.array([prev_conf])

        mat = (current_conf == prev_conf)
        cmp = mat.all()
        if not cmp:
            min_confs_num = 3
            max_dist = np.max(np.abs(current_conf - prev_conf))
            confs_num = int(max_dist / self.resolution)
            confs_num = max(confs_num, min_confs_num)
            confs = np.linspace(start=prev_conf, stop=current_conf, num=confs_num, endpoint=True)
        else:
            confs_num = 1
            confs = np.array([prev_conf])

        mat = (current_conf_camera == prev_conf_camera)
        cmp = mat.all()
        if not cmp:
            min_confs_num = 3
            max_dist_cam = np.max(np.abs(current_conf_camera - prev_conf_camera))
            confs_num_cam = max(confs_num, min_confs_num)
            confs_camera = np.linspace(start=prev_conf_camera, stop=current_conf_camera, num=confs_num, endpoint=True)
        else:
            confs_num_cam = 1
            confs_camera = np.array([prev_conf_camera])

        if confs_num > confs_num_cam:
            diff = confs_num - confs_num_cam
            # add diff elements to the confs_camera array which are equal to the last element
            duplicated_conf = np.tile(confs_camera[-1], (diff, 1))
            confs_camera = np.vstack((confs_camera, duplicated_conf))

        elif confs_num < confs_num_cam:
            diff = confs_num_cam - confs_num
            # add diff elements to the confs array which are equal to the last element
            duplicated_conf = np.tile(confs[-1], (diff, 1))
            confs = np.vstack((confs, duplicated_conf))

        tot_confs = zip(confs, confs_camera)
        for conf in tot_confs:
            collision = self.is_in_collision(conf[0], conf[1])
            if collision:
                return False, ([], [])
        # confs - the configurations of the task robots
        # confs_camera - the configurations of the camera robot
        return True, (confs, confs_camera)

    def simple_is_in_collision(self, conf):
        return self.is_in_collision([0, -pi / 2, 0, -pi / 2, 0, 0], conf)

    def simple_lp(self, conf1, conf2):
        home_conf = np.array([0, -pi / 2, 0, -pi / 2, 0, 0])
        return self.local_planner((home_conf, home_conf), (conf1, conf2))[0]

    def edge_cost(self, conf1, conf2):
        """
        Returns the Edge cost- the cost of transition from configuration 1 to configuration 2
        @param conf1 - configuration 1
        @param conf2 - configuration 2
        """
        diff = np.abs(conf1 - conf2)
        return max(diff) / 0.5

    def get_intervals(self, config):
        intervals = intervals_runner.create_visibility_intervals(self.intervals_path, config, self)
        return intervals
