#! /usr/bin/env python3

import argparse

__author__ = 'Kevin C. Gall'

def main(args):
    map_files = args.maps
    goals = args.goals
    prob = args.intervention_probability
    out_path = args.path




if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate scenes for gridmaps.')

    parser.add_argument('maps', nargs='+', required=True,
                        help='List of paths to input domain maps. MUST BE RELATIVE PATH to cwd')
    parser.add_argument('-n', '--num-scenes', type=int, required=True,
                        help='Number of scenes to create for each passed map')
    parser.add_argument('-g', '--goals', help='total number of goals to generate', type=int, default=1)
    parser.add_argument('-i', '--intervention-probability', type=float, default=0.1,
                        help='The probability that any given clear cell will have an intervention available')
    parser.add_argument('-p', '--path',
                        help='directory path to save the scenes. MUST BE RELATIVE PATH to cwd', default='./gridmap')
