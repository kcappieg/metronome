#!/usr/bin/env python3

import argparse
from time import sleep
from datetime import datetime

import pygame
from pygame import display, draw, font, event, Rect, Surface

__author__ = 'Kevin C. Gall'

# Playback visualizer for Grid World runs.
# There is some overlap w/ metronome_visualizer.py in this dir, but the focus of this
# visualizer is to play back runs of algorithms

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

LINE_DELIMETER = '\n'

CELL_SIZE = 30
OBSERVER_SIZE = int(CELL_SIZE * 0.8)
AGENT_SIZE = int(CELL_SIZE * 0.6)

MIN_SCREEN_SIZE = (500, 500)
SCREEN_PADDING = 15
GRID_LOC = (SCREEN_PADDING, SCREEN_PADDING)

OBSTACLE_COLOR = BLACK
AGENT_COLOR = RED
OBSERVER_COLOR = BLUE
GOAL_COLOR = GREEN

SPEED = 500  # ms per frame


def timedelta_to_milliseconds(td):
    return td.total_seconds() * 1000


# All methods involving interaction return False if the
# user quit
class Visualizer:
    def __init__(self, verbose):
        self._verbose = verbose

    def _print(self, *args):
        if self._verbose:
            print(*args)

    def init(self, source_path):
        self._source_file = open(source_path, 'r')

        self._print('init')
        self._iteration = 0
        self._last_update_time = datetime.now()
        self._paused = False

        pygame.init()
        self._init_flyweights()

        first_frame = self._read_frame()
        if first_frame['eof']:
            raise Exception('Empty grid-viz file')
        self._last_frame = first_frame

        self._goals = first_frame['goals']

        self._grid_surface = Surface(first_frame['frame_size'])
        self._screen_size = self._get_screen_size(first_frame['frame_size'])

        self._print('screen size:', self._screen_size)

        # font / text init
        self._iteration_msg_location = (
            SCREEN_PADDING,
            SCREEN_PADDING + self._grid_surface.get_size()[1] + 3
        )

        self._title_font = font.SysFont(None, 30)
        self._small_msg_font = font.SysFont(None, 12)

        # pre-render first frame onto grid surface
        self._render_frame(first_frame)

    def render_title_screen(self):
        if self._source_file is None:
            raise Exception('Visualizer not initialized')

        screen = display.set_mode(self._screen_size)
        txt_img = self._title_font.render('Press ENTER to start', True, WHITE)

        screen.blit(txt_img, (20, 20))
        display.update()

        if not self._get_continue_signal():
            self.quit()
            return False

        self._show_grid()
        return True

    def next_frame(self, speed, single_frame=False):
        self._print('In next frame')
        now = datetime.now()
        ms_since_update = timedelta_to_milliseconds(now - self._last_update_time)

        if ms_since_update < speed:
            self._print('sleeping for', speed - ms_since_update, 'ms')
            sleep((speed - ms_since_update) / 1000)

        for evt in event.get():
            if evt.type == pygame.QUIT:
                self._print('Quit event')
                self.quit()
                return False

            if evt.type == pygame.KEYUP:
                # toggle pause
                if evt.key == pygame.K_p:
                    self._print('p pressed')
                    self._paused = not self._paused

                # enter always unpauses if applicable
                if evt.key == pygame.K_RETURN or evt.key == pygame.K_KP_ENTER:
                    self._print('return pressed')
                    self._paused = False

        self._print('event queue flushed')

        # paused, so go away
        if self._paused:
            self._print('Paused')
            return True

        self._iteration += 1
        frame = self._read_frame()
        if frame['eof']:
            self._print('Finished reading file')
            self._render_finished_msg()
            return False
        self._last_frame = frame

        self._render_frame(frame)
        self._show_grid()

        if single_frame:
            self._paused = True

        return True

    def quit(self, wait_for_signal=False):
        if wait_for_signal:
            self._print('Waiting for shutdown signal')
            self._get_continue_signal()

        self._print('Shutting down visualizer')
        display.quit()
        self._source_file.close()
        self._source_file = None

    def _init_flyweights(self):
        full_cell_rect = Rect(0, 0, CELL_SIZE, CELL_SIZE)

        obstacle = Surface((CELL_SIZE, CELL_SIZE))
        draw.rect(obstacle, OBSTACLE_COLOR, full_cell_rect)

        goal = Surface((CELL_SIZE, CELL_SIZE))
        draw.rect(goal, GOAL_COLOR, full_cell_rect)

        agent_rect = Rect(0, 0, AGENT_SIZE, AGENT_SIZE)
        agent = Surface((AGENT_SIZE, AGENT_SIZE))
        draw.rect(agent, AGENT_COLOR, agent_rect)

        observer_rect = Rect(0, 0, OBSERVER_SIZE, OBSERVER_SIZE)
        observer = Surface((OBSERVER_SIZE, OBSERVER_SIZE))
        draw.rect(observer, OBSERVER_COLOR, observer_rect)

        self._obstacle = obstacle
        self._goal = goal
        self._agent = agent
        self._observer = observer

    def _read_frame(self):
        """Reads a frame from the file. Returns dict of rectangles to be rendered
    Detects uneven line-lengths and raises exception"""
        self._print('reading frame')
        line = self._source_file.readline()

        while line == '\n':
            line = self._source_file.readline()

        if line == '':
            return {
                'eof': True
            }

        obstacles = []  # holds all obstacle pts
        goals = []  # holds all goal pts
        agent = None
        observer = None

        agent_offset = (CELL_SIZE - AGENT_SIZE) / 2
        observer_offset = (CELL_SIZE - OBSERVER_SIZE) / 2

        width = -1

        rows = 0
        while line != '' and line != '\n':
            column = 0
            for char in line:
                if char == '\n':
                    continue

                left, top = (CELL_SIZE * column, CELL_SIZE * rows)
                agent_left, agent_top = (
                    left + agent_offset,
                    top + agent_offset,
                )
                observer_left, observer_top = (
                    left + observer_offset,
                    top + observer_offset,
                )

                if char == '#':
                    obstacles.append((left, top))
                elif char == '*':
                    goals.append((left, top))
                elif char == '@':
                    agent = (agent_left, agent_top)
                elif char == 'O' or char == 'o':
                    observer = (observer_left, observer_top)

                column += 1

            if width < 0:
                width = column
            elif width != column:
                self._print(width, column, rows)
                raise Exception('Uneven frame')

            rows += 1
            line = self._source_file.readline()

        # if observer is obscured by agent, grab observer's last known location
        observer_obscured = False
        if observer is None and self._last_frame is not None and self._last_frame['observer'] is not None:
            observer_obscured = True
            if self._last_frame['observer_obscured']:
                observer = self._last_frame['observer']
            else:
                observer = (agent[0] - agent_offset + observer_offset,
                            agent[1] - agent_offset + observer_offset)

        return {
            'eof': False,
            'obstacles': obstacles,
            'goals': goals,
            'agent': agent,
            'observer': observer,
            'observer_obscured': observer_obscured,
            'frame_size': (CELL_SIZE * (width), CELL_SIZE * (rows))
        }

    def _get_screen_size(self, frame_size):
        x, y = frame_size
        padding = 2 * SCREEN_PADDING

        if frame_size[0] < MIN_SCREEN_SIZE[0] - padding:
            x = MIN_SCREEN_SIZE[0]
        else:
            x += padding

        if frame_size[1] < MIN_SCREEN_SIZE[1] - padding:
            y = MIN_SCREEN_SIZE[1]
        else:
            y += padding

        return (x, y)

    def _get_continue_signal(self):
        self._print('Getting continue signal')
        while True:
            evt = event.wait()

            if evt.type == pygame.QUIT:
                return False

            if evt.type == pygame.KEYUP:
                if evt.key == pygame.K_RETURN or evt.key == pygame.K_KP_ENTER:
                    return True

    def _render_frame(self, frame_info):
        """Renders frame onto grid surface"""
        self._print('rendering frame onto grid surface')
        self._grid_surface.fill(WHITE)  # erase all previous

        blit_info = [(self._obstacle, obstacle_loc)
                     for obstacle_loc in frame_info['obstacles']]

        blit_info += [(self._goal, goal_loc)
                      for goal_loc in self._goals]

        if frame_info['observer'] is not None:
            blit_info.append((self._observer, frame_info['observer']))

        if frame_info['agent'] is not None:
            blit_info.append((self._agent, frame_info['agent']))

        self._grid_surface.blits(blit_info)

    def _show_grid(self):
        self._print('updating to show currend grid surface')

        screen = display.get_surface()
        iteration_msg = self._small_msg_font.render(
            f'Iteration {self._iteration}', True, WHITE)

        screen.fill(BLACK)  # wipe everything
        screen.blits([
            (self._grid_surface, GRID_LOC),
            (iteration_msg, self._iteration_msg_location)
        ])

        display.update()
        self._last_update_time = datetime.now()

    def _render_finished_msg(self):
        screen = display.get_surface()
        finish_msg = self._title_font.render(
            'Playback finished', True, BLUE)

        screen.blit(
            finish_msg, (SCREEN_PADDING * 2, SCREEN_PADDING * 2)
        )
        display.update()


def main(args):
    viz = Visualizer(False)
    viz.init(args.source)

    not_finished = viz.render_title_screen()

    while not_finished:
        not_finished = viz.next_frame(SPEED)

    viz.quit(True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize Metronome outputs')
    parser.add_argument('source', type=str,
                        help='Source path for the visualization document')

    args = parser.parse_args()
    main(args)
