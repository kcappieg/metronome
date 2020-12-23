#! /usr/bin/env python3

import argparse
import os
import shutil
import numpy as np
from typing import List
from filter_domains import filter_domains
import rand_seed

__author__ = 'Kevin C. Gall'

def read_map_file(input_file):
    map_info = {
        'width': int(input_file.readline()),
        'height': int(input_file.readline()),
        'obstacles': set()
    }

    y = 0
    line = input_file.readline()
    while line != '':
        if line == '\n':
            continue

        for x in range(map_info['width']):
            if line[x] == '#':
                map_info['obstacles'].add((x, y))

        y = y+1
        line = input_file.readline()

    return map_info


def create_gridworld_scene(base_map_info, num_goals, intervention_prob):
    """Creates a scene with random start locations for observer and subject"""
    np.random.seed(rand_seed.next_seed())
    full_scene = base_map_info.copy()
    occupied = full_scene['obstacles'].copy()

    def get_point():
        return (
            np.random.randint(full_scene['width']),
            np.random.randint(full_scene['height'])
        )

    # randomly set observer and subject start locations

    # agent
    pt = get_point()
    while pt in occupied:
        pt = get_point()
    occupied.add(pt)
    full_scene['agent'] = pt

    # observer
    while pt in occupied:
        pt = get_point()
    occupied.add(pt)
    full_scene['observer'] = pt

    # randomly set goals
    full_scene['goals'] = set()
    for _ in range(num_goals):
        while pt in occupied:
            pt = get_point()
        occupied.add(pt)
        full_scene['goals'].add(pt)

    # randomly set remaining cells to be intervention cells or not
    full_scene['interventions'] = []
    for x in range(full_scene['width']):
        for y in range(full_scene['height']):
            pt = (x, y)
            if pt not in occupied and np.random.rand() < intervention_prob:
                full_scene['interventions'].append(pt)

    return full_scene



def write_gridworld_instance(full_map_info, out_file_path, tmp_alt_path):
    real_instance = open(out_file_path, 'w')
    temp_instance = open(tmp_alt_path, 'w')

    preamble = str(full_map_info['width'])+'\n'+str(full_map_info['height'])+'\n'
    world = preamble
    alt_world = preamble

    for y in range(0, full_map_info['height']):
        for x in range(0, full_map_info['width']):
            pt = (x, y)

            if pt == full_map_info['agent']:
                world += '@'
                alt_world += '_'
            elif pt == full_map_info['observer']:
                world += 'o'
                alt_world += '@'
            elif pt in full_map_info['goals']:
                world += '*'
                alt_world += '*'
            elif pt in full_map_info['obstacles']:
                world += '#'
                alt_world += '#'
            elif pt in full_map_info['interventions']:
                world += 'A'
                alt_world += 'A'
            else:
                world += '_'
                alt_world += '_'
        world += '\n'
        alt_world += '\n'

    real_instance.write(world)
    temp_instance.write(alt_world)

    real_instance.close()
    temp_instance.close()


def main(args):
    map_files: List[str] = args.maps
    resource_dir = args.resource_dir
    num_goals = args.goals
    num_scenes = args.num_scenes
    intervention_prob = args.intervention_probability
    out_path = args.path

    if not os.path.exists(out_path):
        os.makedirs(out_path)
    alt_dir = os.path.join('./temp', out_path)
    if not os.path.exists(alt_dir):
        os.makedirs(alt_dir)

    for map_file in map_files:
        alt_instance_paths = []
        ext_index = map_file.rindex('.map')
        base_filename = map_file[:ext_index] + '-scn'
        with open(os.path.join(resource_dir, map_file)) as map_file:
            map_info = read_map_file(map_file)

        for i in range(num_scenes):
            out_file_path = os.path.join(out_path, base_filename + str(i) + '.vw')
            tmp_alt_path = os.path.join(alt_dir, base_filename + str(i) + '.vw')

            domain_instance = create_gridworld_scene(map_info, num_goals, intervention_prob)
            write_gridworld_instance(domain_instance, out_file_path, tmp_alt_path)
            alt_instance_paths.append(tmp_alt_path)

        if args.filter:
            domain_type = 'VACUUM_WORLD' if num_goals > 1 else 'GRID_WORLD'
            print('Checking for observer access to goals')
            passed = filter_domains(alt_instance_paths, base_filename, domain_type, out_path, write=False)
            domain_instance_paths = ['/' + domain_path[5:] for domain_path in passed]
            print('Filtering from remaining')
            filter_domains(domain_instance_paths, base_filename, domain_type, out_path, write=True)

    shutil.rmtree(alt_dir)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate scenes for gridmaps.')

    parser.add_argument('maps', nargs='+',
                        help='List of filenames to input domain maps')
    parser.add_argument('-r', '--resource-dir', type=str, required=True,
                        help='Relative path to the directory that contains the map files')
    parser.add_argument('-n', '--num-scenes', type=int, required=True,
                        help='Number of scenes to create for each passed map')
    parser.add_argument('-g', '--goals', help='total number of goals to generate', type=int, default=1)
    parser.add_argument('-i', '--intervention-probability', type=float, default=0.1,
                        help='The probability that any given clear cell will have an intervention available')
    parser.add_argument('-p', '--path',
                        help='directory path to save the scenes. MUST BE RELATIVE PATH to cwd', default='./gridmap')
    parser.add_argument('-f', '--filter', default=None, action='store_true',
                        help='Filter generated domains to only solvable. Assumes a previous build of Metronome. Executes A_STAR on each domain.')

    main(args=parser.parse_args())