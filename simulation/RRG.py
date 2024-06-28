import operator
import numpy as np
import pickle as pkl
import os

from building_blocks import Building_Blocks

TASK_TIME = 43.5


class Vertex:
    def __init__(self, config, intervals):
        self.config = config
        self.intervals = intervals


class RRG:
    def __init__(self, bb: Building_Blocks, graph_number=0):
        self.bb = bb
        self.vertices = {}
        self.edges = {}

        self.graph_number = graph_number

        self.step_size = 100
        print(f'step size is: {self.step_size}')

        # create dir
        if not os.path.exists(f'graph_{self.graph_number}'):
            os.makedirs(f'graph_{self.graph_number}')

    def run(self):
        start_config = np.array([-1.2, -np.pi / 2, 0, -np.pi / 2, 0, 0])
        self.add_vertex(start_config, intervals=[])
        while len(self.edges) < 1000:
            for _ in range(10):
                self.extend()

            self.save_graph(f'graph_{self.graph_number}/graph_{len(self.vertices.keys()) - 1}.pkl')

    def extend(self):
        # sample random state
        new_config, intervals = self.sample_new_config()

        # find k nearest neighbors
        near_ids, near_configs = self.get_k_nearest_neighbors(new_config, k=max(1, int(np.log2(len(self.vertices)))))

        # add new config to graph
        new_config_idx = self.add_vertex(new_config, intervals)

        # add edges
        for near_config_idx, near_config in zip(near_ids, near_configs):
            if self.bb.simple_lp(near_config, new_config):
                self.add_edge(near_config_idx, new_config_idx)

    def sample_new_config(self):
        first_time, new_config, nearest_config, intervals = True, None, None, []
        i = 0
        require_interval = np.random.rand() < 0.05

        while first_time or \
            self.bb.simple_is_in_collision(new_config) or \
            not self.bb.simple_lp(nearest_config, new_config) or \
            (require_interval and len(intervals) == 0):
            first_time = False

            # sample random state
            rand_config = np.random.uniform(low=-np.pi, high=np.pi, size=6)
            # find nearest neighbor
            _, nearest_config = self.get_nearest_config(rand_config)
            # steer towards the new point
            new_config = self.steer(nearest_config, rand_config)
            # compute intervals
            intervals = self.bb.get_intervals(new_config)
            i += 1

        print(f'sampling took {i} iterations - require interval - {require_interval}')
        return new_config, intervals

    def steer(self, near_config, rand_config):
        """
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        """
        # Euclidean distance between the two points
        dist = self.bb.edge_cost(near_config, rand_config)

        # return a point lies on the vector towards the new point
        if dist > self.step_size:
            return self.step_size * (rand_config - near_config) / dist
        # return the point itself
        else:
            return rand_config

    def get_graph_intervals(self):
        intervals = {vid: vertex.intervals for vid, vertex in self.vertices.items()}
        return self.edges, intervals

    def save_graph(self, filename):
        graph, intervals = self.get_graph_intervals()

        with open(f'{filename}', 'wb') as f:
            pkl.dump((graph, intervals), f)

        filename = filename.replace('.pkl', '.txt')
        with open(f'{filename}', 'w') as f:
            for u in graph.keys():
                f.write(f'{u} {len(graph[u])} ')

                for v, w in graph[u]:
                    f.write(f'{v} {w} ')

                f.write('\n')

            f.write('-\n')

            for node, interval in intervals.items():
                f.write(f'{node} {len(interval)}')
                for start, end in interval:
                    f.write(f' {start} {end}')
                f.write('\n')

            f.write('-\n')

        print(f'saved: {filename}')

    def add_vertex(self, config, intervals):
        """
        Add a state to the tree.
        @param config Configuration to add to the tree.
        @param intervals of the configuration.
        """
        vid = len(self.vertices)

        self.vertices[vid] = Vertex(config=config, intervals=intervals)
        self.edges[vid] = []

        return vid

    def add_edge(self, sid, eid):
        """
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        """
        total_time = TASK_TIME
        edge_cost = self.compute_distance(sid, eid) / total_time

        self.edges[sid].append((eid, edge_cost))
        self.edges[eid].append((sid, edge_cost))

    def compute_distance(self, sid, eid):
        config1 = self.vertices[sid].config
        config2 = self.vertices[eid].config
        distance = self.bb.edge_cost(config1, config2)
        return distance

    def get_nearest_config(self, config):
        """
        Find the nearest vertex for the given config and returns its state index and configuration
        @param config Sampled configuration.
        """
        # compute distances from all vertices
        dists = []
        for _, vertex in self.vertices.items():
            dists.append(self.bb.edge_cost(config, vertex.config))

        # retrieve the id of the nearest vertex
        if len(dists) > 0:
            vid, _ = min(enumerate(dists), key=operator.itemgetter(1))

            return vid, self.vertices[vid].config
        else:
            return None, None

    def get_k_nearest_neighbors(self, config, k):
        """
        Return k-nearest neighbors
        @param config Sampled configuration.
        @param k Number of nearest neighbors to retrieve.
        """
        if k == 1:
            vid, config = self.get_nearest_config(config)
            return [vid], [config]

        dists = []
        for _, vertex in self.vertices.items():
            dists.append(self.bb.edge_cost(config, vertex.config))

        dists = np.array(dists)
        knn_ids = np.argpartition(dists, k)[:k]

        return knn_ids.tolist(), [self.vertices[vid].config for vid in knn_ids]
