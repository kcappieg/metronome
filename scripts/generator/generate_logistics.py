#! /usr/bin/env python3

import os
import argparse
from math import floor, ceil
import numpy as np
from collections import deque

__author__ = 'Kevin C. Gall'


# Network density given as actual / potential connections
# Potential connections = (n * (n - 1)) / 2
# Note that with density, the network will be created such that every node
# is reachable based on the desired topology. If the network has not reached
# requested density, it is treated as an upper bound on network density

def main(args):
    locations = args.locations
    goals = args.goals
    total = args.total
    density = args.density
    topology = args.topology
    clusters = args.clusters
    connection_distance = args.connection_distance
    trucks = args.trucks
    packages = args.packages

    out_path = args.path
    verbose = args.verbose

    if verbose:
        print(total, 'instances requested')
        print('Chose topology', topology)

    network_builder = None

    if topology == 'cluster':
        network_builder = ClusterNetwork(locations, density, clusters)
    elif topology == 'cycle':
        network_builder = CycleNetwork(locations, density)
    elif topology == 'geometric':
        network_builder = GeometricNetwork(locations, connection_distance)

    if topology == 'cluster':
        file_name = f'{clusters}clusters_{density}dense'
    elif topology == 'geometric':
        file_name = f'geometric_{connection_distance}dist'
    else:
        file_name = f'{topology}_{density}dense'

    file_name += f'_{goals}goal_{locations}loc_{packages}pkg_{trucks}trk'

    for i in range(total):
        # create network topology
        instance = f'Locations:{locations}\n'
        instance += network_builder.generate_network() + '\n'

        # trucks
        instance += f'Trucks:{trucks}\n'
        for trkId in range(trucks):
            instance += f'{trkId} {np.random.randint(0, locations)}\n'

        # packages
        instance += f'\nPackages:{packages}\n'
        for pkgId in range(packages):
            instance += f'{pkgId} {np.random.randint(0, locations)}\n'

        # goal hypotheses
        instance += '\nGoals:\n'
        for _ in range(goals):
            hyp = list()
            # weighting 2-goal combos higher than others
            num_conditions = np.random.choice([1, 2, 3], p=[0.25, 0.5, 0.25])

            for pkg in np.random.choice(packages, num_conditions, replace=False):
                hyp.append(f'({pkg} {np.random.randint(0, locations)})')

            instance += ','.join(hyp) + '\n'

        # observer
        instance += f'\nObserver:{np.random.randint(0, locations)}\n'

        full_file_path = os.path.join(out_path, f'{file_name}_{i}.logistics')
        with open(full_file_path, 'w') as file:
            file.write(instance)


# Abstract class
class Network:
    def __init__(self, locations, density):
        self._num_locations = locations
        self._location_list = np.arange(locations)
        # Potential Connections
        pc = (locations * (locations - 1)) / 2
        self._max_connections = floor(density * pc)

    def generate_base_network(self):
        '''Generate the base network
        Must return set of tuples for duplicate detection
        Responsible for ensuring base network connects all nodes'''
        raise Exception('Must be implemented by child class')

    def generate_network(self):
        network = self.generate_base_network()

        i = 0
        while len(network) / 2 < self._max_connections:
            connection = np.random.choice(self._location_list, 2, replace=False)
            # if repeat, won't add it
            network.add((connection[0], connection[1]))
            network.add((connection[1], connection[0]))

            i += 1
            # failsafe
            if i >= 500:
                break

        # get rid of duplicate edges (i.e. reversed edges)
        network_no_dupes = set()
        for edge in network:
            if not (edge[1], edge[0]) in network_no_dupes:
                network_no_dupes.add(edge)

        max_cost = 5  # arbitrary
        network_desc = ''
        for edge, cost in zip(network_no_dupes, np.random.randint(1, max_cost + 1, len(network_no_dupes))):
            network_desc += f'{edge[0]} {edge[1]} {cost}\n'

        return network_desc


class ClusterNetwork(Network):
    def __init__(self, locations, density, clusters):
        super().__init__(locations, density)
        self._clusters = clusters

    def generate_base_network(self):
        '''Creates random clusters of nodes.
        Ensures there is a path through each cluster so
        that every node is connected to every other node'''
        # using choice like a shuffle, just not in place
        shuffled_list = np.random.choice(self._location_list, self._num_locations, replace=False)

        max_cluster_size = ceil(self._num_locations / self._clusters)
        cluster_leaf_lists = list()
        current_leaf_list = None

        network = set()
        cluster_root = -1
        for idx, loc in enumerate(shuffled_list):
            if idx % max_cluster_size == 0:
                cluster_root = loc
                current_leaf_list = list()
                cluster_leaf_lists.append(current_leaf_list)
                continue

            # keeping track of leaves so we can connect clusters later
            current_leaf_list.append(loc)
            edge = (cluster_root, loc)
            alt = (edge[1], edge[0])
            network.add(edge)
            network.add(alt)

        # create a connection through clusters, if applicable
        # this ensures all nodes are reachable
        if self._clusters > 1:
            for i in range(self._clusters - 1):
                edge = (
                    np.random.choice(cluster_leaf_lists[i]),
                    np.random.choice(cluster_leaf_lists[i + 1])
                )
                alt = (edge[1], edge[0])
                network.add(edge)
                network.add(alt)

        return network


class CycleNetwork(Network):
    def generate_base_network(self):
        '''Creates a random-path cycle through the nodes'''
        shuffled_list = np.random.choice(self._location_list, self._num_locations, replace=False)

        network = set()
        for i in range(self._num_locations):
            next_loc = (i + 1) if i+1 < self._num_locations else 0

            edge = (
                shuffled_list[i],
                shuffled_list[next_loc]
            )
            alt = (edge[1], edge[0])
            network.add(edge)
            network.add(alt)

        return network

class GeometricNetwork:
    def __init__(self, locations, connection_distance):
        self._num_locations = locations
        self._max_dist_sq = connection_distance ** 2

    def get_geometric_graph(self):
        locations = [
            (np.random.random_sample(), np.random.random_sample())
            for _ in range(self._num_locations)
        ]
        # naive n^2 connection. Shouldn't be a problem w/ amount of locations we'll generate

        edge_lookup = [list() for _ in range(self._num_locations)]
        for i in range(self._num_locations - 1):
            for j in range(i+1, self._num_locations):
                dist_sq = euclidean_dist_sq(locations[i], locations[j])
                if dist_sq <= self._max_dist_sq:
                    edge = (i, j, dist_sq)
                    edge_lookup[i].append(edge)
                    edge_lookup[j].append(edge)

        return edge_lookup

    def generate_network(self):
        connected_network = None
        for try_idx in range(10):
            edge_lookup = self.get_geometric_graph()

            # run reachability search
            closed = {0}
            q = deque([0])

            while len(q) > 0:
                loc = q.popleft()
                for edge in edge_lookup[loc]:
                    other = edge[0] if edge[0] != loc else edge[1]
                    if other not in closed:
                        q.append(other)
                        closed.add(other)

            # not fully connected. Throw out and try again
            if len(closed) < self._num_locations:
                continue

            # otherwise build network
            connected_network = set()
            for edge_list in edge_lookup:
                for edge in edge_list:
                    connected_network.add(edge)

        if connected_network is None:
            raise Exception('Could not generate connected geometric graph with given distance. '
                            'Try increasing connection distance')

        network_desc = ''
        for edge in connected_network:
            cost = ceil(5.0 * (edge[2] / self._max_dist_sq))
            network_desc += f'{edge[0]} {edge[1]} {cost}\n'

        return network_desc


        return connected_network


def euclidean_dist_sq(pt1, pt2):
    '''Helper - Euclidean dist between points, squared'''
    return ((pt1[0] - pt2[0]) ** 2) + ((pt1[1] - pt2[1]) ** 2)


if __name__ == '__main__':
    # Begin Argument Definition

    parser = argparse.ArgumentParser(description='Generate Logistics instances. Can choose between '
                                                 'topologies: cluster or cycle')

    parser.add_argument('-l', '--locations', help='Number of locations', type=int, default=3)
    parser.add_argument('-g', '--goals', help='total number of possible goals to generate', type=int, default=1)
    parser.add_argument('-t', '--total', help='total number of instances to generate', type=int, default=1)
    parser.add_argument('-s', '--density', help='Max density of network. Not guaranteed, simply a guide to the '
                                                'generator. If topology is geometric, this value is ignored.',
                        type=float, default=0)
    parser.add_argument('--topology', help='Cluster creates groups with central nodes. Cycle distributes connections'
                                           'more evenly throughout the network. Geometric is based on random'
                                           'sampling within unit square',
                        choices=['cluster', 'cycle', 'geometric'], default='geometric')
    # TODO: add option for cost range. Just using 1 - 5 for now.
    # TODO: add option for goal fluent count. Just using 1 - 3 for now.
    parser.add_argument('-d', '--connection-distance',
                        help='If topology is geometric, defines the connection distance between (0,1)',
                        type=float, default=0.2)
    parser.add_argument('-c', '--clusters', help='If topology is cluster, how many clusters to create',
                        type=int, default=1)
    parser.add_argument('-k', '--trucks', help='Number of trucks', type=int, default=1)
    parser.add_argument('-a', '--packages', help='Number of packages', type=int, default=3)

    parser.add_argument('-p', '--path',
                        help='directory path to save the worlds. MUST BE RELATIVE PATH to cwd. That is a known issue, '
                             'but no time to fix',
                        default='./logistics')
    parser.add_argument('-v', '--verbose', help='increase output verbosity', action='store_true')

    # End argument definition

    main(args=parser.parse_args())