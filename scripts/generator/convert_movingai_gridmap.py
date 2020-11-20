#!/usr/bin/env python3

# Converts movingai.com 2d gridmaps into ascii format used
# by this repository
# Currently only converts the base map, does not take scenario,
# meaning outputs will not include agent/goal.
#
# Supports scaling down maps. To deal with scaling down obstacles,
# if a dimension (up or down) of a group of cells being scaled down
# is fully blocked, then the scaled down cell is blocked

import os
from collections import deque
import argparse

__author__ = 'Kevin C. Gall'


def main(args):
    input_dir = args.input_dir
    out_path = args.path
    scale_factor = args.factor
    chop = args.chop

    if input_dir is None:
        raise Exception('No input specified')

    for entry in os.scandir(input_dir):
        if not entry.is_file(): continue

        with open(entry.path, 'r') as in_file:
            in_file.readline()  # type octile
            original_height = int(in_file.readline().replace('height ', ''))
            original_width = int(in_file.readline().replace('width ', ''))
            in_file.readline()  # map

            # we're only going to accept even scale-downs for now
            if original_height % scale_factor != 0 or original_width % scale_factor != 0:
                raise Exception('Scale factor does not evenly divide height or width')

            height = int((original_height / scale_factor) / chop)
            width = int((original_width / scale_factor) / chop)

            world = f'{width}\n{height}\n'
            for h in range(0, original_height, scale_factor):
                if h > original_height / chop:
                    break
                line_block = [convert_chars(in_file.readline()) for _ in range(scale_factor)]
                for w in range(0, original_width, scale_factor):
                    if w >= original_width / chop:
                        break
                    block = [line[w:w+scale_factor] for line in line_block]
                    world += condense_block(block)

                world += '\n'

            with open(os.path.join(out_path, entry.name), 'w') as out_file:
                out_file.write(world)


def convert_chars(line):
    return line.replace('.', '_') \
        .replace('G', '_') \
        .replace('@', '#') \
        .replace('O', '#') \
        .replace('T', '#') \
        .replace('S', '_') \
        .replace('W', '#')


def condense_block(block):
    """Detect if either vertical or horizontal traversal is blocked through this cell"""
    if len(block) == 1 and len(block[0]) == 1:
        return block[0][0]

    # horizontal
    # queue elements: point (h,w)
    queue = deque()
    for h in range(len(block)):
        if block[h][0] == '#':
            queue.append((h, 0))

    # loop through laterally to try to find a blocked segment
    while len(queue) > 0:
        if extend_path(queue.pop(), block, queue, True):
            return '#'

    for w in range(len(block[0])):
        if block[0][w] == '#':
            queue.append((0, w))

    # loop through vertically to try to find a blocked segment
    while len(queue) > 0:
        if extend_path(queue.pop(), block, queue, False):
            return '#'

    return '_'


def extend_path(start_el, block, queue, is_lateral):
    """Extends path through block and returns true if the way is blocked"""
    next_el = start_el

    h = len(block)
    w = len(block[0])
    while next_el is not None:
        current = next_el
        next_el = None
        for i in range(-1, 2):
            add_to_height = i if is_lateral else 1
            add_to_width = i if not is_lateral else 1

            candidate = (current[0] + add_to_height, current[1] + add_to_width)
            if candidate[0] < 0 or candidate[0] >= h or candidate[1] < 0 or candidate[1] >= w:
                continue

            if block[candidate[0]][candidate[1]] == '#':
                if (is_lateral and candidate[1] == w - 1) or (not is_lateral and candidate[0] == h -1):
                    return True
                elif next_el is None:
                    next_el = candidate
                else:
                    queue.append(candidate)

    return False


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert GridMap maps from movingai to ascii format '
                                                 'used by this repo')

    parser.add_argument('-f', '--factor', help='factor to scale down the map', type=int, default=1)
    parser.add_argument('-c', '--chop', help='factor by which to chop final map', type=int, default=1)
    parser.add_argument('-p', '--path',
                        help='directory path to save the scaled maps. MUST BE RELATIVE PATH to cwd',
                        default='./gridmap')
    parser.add_argument('-i', '--input-dir', help='directory where input gridmaps are found')
    parser.add_argument('-v', '--verbose', help='increase output verbosity', action='store_true')

    main(args=parser.parse_args())
