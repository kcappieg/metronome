#! /usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import rand_seed
from filter_domains import filter_domains

__author__ = 'Kevin C. Gall'


# Good configs:
# python3 generate_grid_world.py 1500 1500 1000 -o 0.0006 -s corridors -c 0.07 -e 2
#   around 10% solvable


def generate_goals(goals, width, height, start, observer_start=None):
    goal_set = set()
    if goals > 1:
        while len(goal_set) < goals:
            goal = (np.random.randint(0, width - 1), np.random.randint(0, height - 1))
            if goal != start and goal != observer_start:
                goal_set.add(goal)
    else:
        goal_set.add((width - 2, height - 2))

    return goal_set


class SingleObstacleStrategy:
    def __init__(self, width, height, probability, intervention_percentage):
        self.width = width
        self.height = height
        self.probability = probability
        self.intervention_percentage = intervention_percentage
        self._observer_start = None

    def generate_goals(self, goals):
        return generate_goals(goals, self.width, self.height, self.get_start(), self._observer_start)

    def get_obstacles(self):
        obstacle_locations = set()
        for y in range(0, self.height):
            for x in range(0, self.width):
                if np.random.random() < self.probability:
                    obstacle_locations.add((x, y))

        return obstacle_locations

    def get_interventions(self, obstacle_locations):
        intervention_locations = set()
        if self.intervention_percentage is 0.0:
            return intervention_locations

        for y in range(0, self.height):
            for x in range(0, self.width):
                if (x, y) not in obstacle_locations and np.random.random() < self.intervention_percentage:
                    intervention_locations.add((x, y))

        return intervention_locations

    def get_start(self):
        return 1, 1

    def get_observer_start(self):
        obs_start = (1, 1)
        while obs_start == (1, 1):
            obs_start = (np.random.randint(0, self.width), np.random.randint(0, self.height))

        self._observer_start = obs_start
        return obs_start

    def reset(self):
        pass


class EnclosureObstacleStrategy:
    def __init__(self, width, height, probability, intervention_percentage, sizeBound, widthFactor=1.0, exits=0,
                 equalLength=False, alignDirections=False):
        self.width = width
        self.height = height
        self.intervention_percentage = intervention_percentage
        self.sizeBound = sizeBound
        self.widthSizeBound = int(max(sizeBound * widthFactor, 1))
        self.exits = exits
        self.equalLength = equalLength
        self.alignDirections = alignDirections
        self.directions = self.get_directions() if alignDirections else None

    def generate_goals(self, goals):
        return generate_goals(goals, self.width, self.height, self.get_start())

    def get_directions(self):
        firstVector = dict()
        midVector = dict()
        lastVector = dict()
        direction = np.random.randint(0, 3)

        if (direction == 0):
            firstVector['x'] = -1
            firstVector['y'] = 0
        elif (direction == 1):
            firstVector['x'] = 1
            firstVector['y'] = 0
        elif (direction == 2):
            firstVector['x'] = 0
            firstVector['y'] = -1
        elif (direction == 3):
            firstVector['x'] = 0
            firstVector['y'] = 1

        lastVector['x'] = -firstVector['x']
        lastVector['y'] = -firstVector['y']

        nextDirection = np.random.randint(0, 1)
        if nextDirection == 0:
            nextDirection = -1

        if firstVector['x'] == 0:
            midVector['x'] = nextDirection
            midVector['y'] = 0
        else:
            midVector['y'] = nextDirection
            midVector['x'] = 0

        return firstVector, midVector, lastVector

    def add_enclosure(self, obstacleTracker, start):
        # Get random size for the enclosure
        firstWallSize = np.random.randint(1, self.sizeBound)
        midWallSize = np.random.randint(1, self.widthSizeBound)
        lastWallSize = firstWallSize + 1 if self.equalLength else np.random.randint(1, self.sizeBound)

        # get the directions - which way does the wall extend from its starting position
        obstacleDirections = self.directions if self.directions != None else self.get_directions()

        firstVector = obstacleDirections[0]
        midVector = obstacleDirections[1]
        lastVector = obstacleDirections[2]

        # get random exits - "holes" in the walls where an agent can pass through
        exitSet = set()
        while len(exitSet) < self.exits:
            exitWall = np.random.randint(0, 1)
            # only the length walls have exits
            if exitWall == 1:
                exitWall = 2

            wallSize = firstWallSize if exitWall == 0 else lastWallSize
            exitLoc = np.random.randint(0, wallSize)

            exitSet.add((exitWall, exitLoc))

        # Add wall spaces to the obstacle tracker
        current = (start[0], start[1])
        for wallIndex, wall in enumerate(
                [(firstVector, firstWallSize), (midVector, midWallSize), (lastVector, lastWallSize)]):
            for i in range(wall[1]):
                if (wallIndex, i) not in exitSet:
                    obstacleTracker.add(current)
                current = (current[0] + wall[0]['x'], current[1] + wall[0]['y'])

    def get_obstacles(self):
        obstacle_locations = set()
        for y in range(0, self.height):
            for x in range(0, self.width):
                if np.random.random() < self.probability:
                    self.add_enclosure(obstacle_locations, (x, y))

        return obstacle_locations

    def get_interventions(self, obstacle_locations):
        raise Exception('Interventions not implemented for Enclosure domains')

    def get_start(self):
        return 1, 1

    def reset(self):
        if self.alignDirections:
            self.directions = self.get_directions()


# Creates domain with uniformly distributed obstacles and tunnels
# through said obstacles which are clear of obstructions.
# Tunnels hug the walls leaving the center of the domain the most
# cluttered with no tunnels through
class TunnelsStrategy:
    def __init__(self, width, height, probability, intervention_percentage, size_bound, stddev):
        self.width = width
        self.height = height
        self.probability = probability
        self.intervention_percentage = intervention_percentage
        self.stddev = stddev
        self.size_bound = size_bound
        self.uniform_generator = SingleObstacleStrategy(width, height, probability)

        # create a bound on tunnels based on height
        self.max_tunnels_per_x = max(int(round(height ** 0.1, 0)), 1)
        self.mean = height / 2

    def get_obstacles(self):
        obstacle_locations = self.uniform_generator.get_obstacles()
        # we overlay tunnels on top of uniformly distributed obstacles
        # First, determine how many tunnels will start at a given x
        # Then, sample from a normal distribution to get the y values
        # Then, generate the tunnel

        for x in range(0, self.width):
            tunnels = np.random.randint(0, self.max_tunnels_per_x + 1)

            if tunnels == 0:
                continue

            centered_y_vals = np.random.normal(self.mean, self.stddev, tunnels)
            # convert y to edge value
            edge_y_vals = []
            for y in centered_y_vals:
                dist_from_mean = int(self.mean - y)  # may round. That's fine
                # height - 3 because:
                #   0 Indexed (1)
                #   each tunnel needs width 3, so give ourselves padding
                reference = self.height - 3 if dist_from_mean < 0 else 0
                edge_y_vals.append(reference + dist_from_mean)

            for y in edge_y_vals:
                walls, path = self.generate_tunnel((x, y))
                # claim the tunnel locations
                self.claimed_tunnel_locations.update(walls)
                self.claimed_tunnel_locations.update(path)

                for loc in walls:
                    obstacle_locations.add(loc)

                for loc in path:
                    if loc in obstacle_locations:
                        obstacle_locations.remove(loc)

        return obstacle_locations

    def get_interventions(self, obstacle_locations):
        raise Exception('Interventions not implemented for Tunnel domains')

    # In order to maintain clear tunnels at the edges, we track every
    # space that is part of a tunnel. If another tunnel would overlap
    # with an existing one, that tunnel is simply not added (i.e. an empty set
    # is returned
    def generate_tunnel(self, start_loc):
        # randomly get tunnel length
        length = np.random.random_integers(1, self.size_bound)
        tunnel_walls = set()
        tunnel_path = set()

        start_x, top_y = start_loc

        for x in range(start_x, start_x + length):
            if x == self.width:
                continue

            top_wall_loc = (x, top_y)
            path_loc = (x, top_y + 1)
            bottom_wall_loc = (x, top_y + 2)

            # collision detection
            if not self.claimed_tunnel_locations.isdisjoint(set([top_wall_loc, path_loc, bottom_wall_loc])):
                return set(), set()

            tunnel_path.add(path_loc)
            tunnel_walls.add(top_wall_loc)
            tunnel_walls.add(bottom_wall_loc)

        # clear the entrance and exit to the tunnel
        if start_x > 0 and start_x + length < self.width:  # edge cases
            entrance = (start_x - 1, top_y + 1)
            exit = (start_x + length, top_y + 1)

            # if entrance and exit aren't clear, don't register tunnel!
            if not self.claimed_tunnel_locations.isdisjoint(set([entrance, exit])):
                return set(), set()

            tunnel_path.add(entrance)
            tunnel_path.add(exit)

        return tunnel_walls, tunnel_path

    def generate_goals(self, goal_count=1):
        # goal_count ignored for this domain
        return set([(self.width - 2, int(self.height / 2))])

    def get_start(self):
        return 1, int(self.height / 2)

    def reset(self):
        # claimed tunnel locations prevent us from overlapping tunnels
        self.claimed_tunnel_locations = set()


def main(args):
    height = args.height
    width = args.width
    total = args.total
    goals = args.goals
    obstacle_percentage = args.obstacle_probability
    intervention_percentage = args.intervention_probability
    observer = args.observer

    seed_skip = args.seed_skip
    rand_seed.skip_n_seeds(seed_skip)
    np.random.seed(rand_seed.next_seed())

    # Size bound for enclosures calculated using power
    size_bound = max(int(height ** 0.7), 1)

    if args.verbose:
        print(args.height)
        print(args.width)
        print(f'Percent chance a cell will start an obstacle: {obstacle_percentage}')
        print(f'{goals} goal(s) will be generated')

    strategy = args.strategy
    domain_builder = None
    if strategy == 'single':
        domain_builder = SingleObstacleStrategy(width, height, obstacle_percentage, intervention_percentage)
        if args.verbose:
            print('Single Cell obstacle strategy')

    elif strategy == 'minima':
        domain_builder = EnclosureObstacleStrategy(width, height, obstacle_percentage, intervention_percentage,
                                                   size_bound)
        if args.verbose:
            print(f'Minima obstacle strategy. Size bound {size_bound}')

    elif strategy == 'corridors':
        domain_builder = EnclosureObstacleStrategy(width, height, obstacle_percentage, intervention_percentage,
                                                   size_bound,
                                                   widthFactor=args.corridor_width_factor,
                                                   exits=args.corridor_exits,
                                                   equalLength=True)
        if args.verbose:
            print(f'Corridors obstacle strategy. Size bound {size_bound}')

    elif strategy == 'corridors-aligned':
        domain_builder = EnclosureObstacleStrategy(width, height, obstacle_percentage, intervention_percentage,
                                                   size_bound,
                                                   widthFactor=args.corridor_width_factor,
                                                   exits=args.corridor_exits,
                                                   equalLength=True,
                                                   alignDirections=True)
        if args.verbose:
            print(f'Aligned Corridors obstacle strategy. Size bound {size_bound}')

    elif strategy == 'tunnels':
        domain_builder = TunnelsStrategy(width, height, obstacle_percentage, intervention_percentage, size_bound,
                                         stddev=args.tunnel_deviation)

        if args.verbose:
            print(
                f'Tunnels strategy. Tunnel size bound {size_bound}, obstacle likelihood {obstacle_percentage}, deviation {args.tunnel_deviation}')

    outPath = args.path

    config_type = strategy
    if config_type == 'single':
        config_type = 'uniform'

    if args.verbose:
        print(f'Generating {total} worlds of type {config_type}')

    if not os.path.exists(outPath):
        os.makedirs(outPath)

    generated_domains = []
    base_domain_name = get_base_domain_name(config_type, width, height)
    for iteration in range(total):
        domain_builder.reset()

        newDomain = base_domain_name + str(iteration)
        completeFile = os.path.join(outPath, newDomain + '.vw')
        generated_domains.append(completeFile)

        aFile = open(completeFile, 'w')

        preamble = str(width) + '\n' + str(height) + '\n'
        world = ''

        obs_start = (None, None)
        if observer:
            obs_start = domain_builder.get_observer_start()

        obs_x, obs_y = obs_start
        start_x, start_y = domain_builder.get_start()
        goal_set = domain_builder.generate_goals(goals)
        obstacle_locations = domain_builder.get_obstacles()
        intervention_locations = domain_builder.get_interventions(obstacle_locations)

        for y in range(0, height):
            for x in range(0, width):
                if (x == start_x) and (y == start_y):
                    world += '@'
                    if (x, y) in goal_set:
                        raise Exception('Goal in same space as start')
                elif (x == obs_x) and (y == obs_y):
                    world += 'o'
                    if (x, y) in goal_set:
                        raise Exception('Goal in same space as observer - cannot happen for domain spec reasons')
                elif (x, y) in goal_set:
                    world += '*'
                elif (x, y) in obstacle_locations:
                    world += '#'
                elif (x, y) in intervention_locations:
                    world += 'A'
                else:
                    world += '_'
            world += '\n'

        if args.verbose:
            print(world)

        aFile.write(preamble + world)

        aFile.close()

    if args.filter:
        domain_type = 'VACUUM_WORLD' if goals > 1 else 'GRID_WORLD'
        filter_domains(generated_domains, base_domain_name, domain_type, '.vw', os.path.join(outPath, 'filtered'))


def get_base_domain_name(strategy, width, height):
    return strategy + str(height) + '_' + str(width) + '-'


if __name__ == '__main__':
    # Begin Argument Definition

    parser = argparse.ArgumentParser(
        description='Generate grid-world instances. Tunnel domains place the agent and goal'
                    ' in the center of the grid and distribute the tunnels weighted toward'
                    ' the edges of the grid.')

    parser.add_argument('height', help='the height of the Vehicle world', type=int)
    parser.add_argument('width', help='the width of the Vehicle world', type=int)
    parser.add_argument('total', help='total number of worlds to generate', type=int)
    parser.add_argument('-g', '--goals', help='total number of goals to generate', type=int, default=1)
    parser.add_argument('-p', '--path',
                        help='directory path to save the worlds. MUST BE RELATIVE PATH to cwd. '
                             'That is a known issue, but no time to fix',
                        default='./gridworld')
    parser.add_argument('-v', '--verbose', help='increase output verbosity', action='store_true')
    parser.add_argument('-o', '--obstacle-probability', default=0.0, type=float,
                        help='probability of obstacle to begin in any given grid cell')
    parser.add_argument('-s', '--strategy', choices=['single', 'minima', 'corridors', 'corridors-aligned', 'tunnels'],
                        default='single',
                        help='obstacle structure strategy for the generated worlds. Defaults to "single". If "corridors-aligned", all corridors will be facing one direction')
    parser.add_argument('-c', '--corridor-width-factor', default=0.05, type=float,
                        help='Factor of the width of a corridor. Only used in the corridors and corridors-aligned strategies')
    parser.add_argument('-e', '--corridor-exits', default=1, type=int,
                        help='If a corridor strategy, defines how many "exits" from the corridor will be generated. Exits appear on either of the length walls')
    parser.add_argument('-d', '--tunnel-deviation', default=10, type=float,
                        help='If tunnels strategy, the standard deviation to be used in the normal distribution for tunnel generation. Higher numbers make it more likely for tunnels to be closer to center')
    parser.add_argument('-f', '--filter', default=None, action='store_true',
                        help='Filter generated domains to only solvable. Assumes a previous build of Metronome. Executes A_STAR on each domain.')
    parser.add_argument('--observer', action='store_true', help='Flag to add observer')
    parser.add_argument('-i', '--intervention-probability', default=0.0, type=float,
                        help='Probability of possible intervention in any given clear cell')
    parser.add_argument('--seed-skip', type=int, default=0,
                        help='If passed, skip this many random seeds')

    # End argument definition

    main(args=parser.parse_args())
