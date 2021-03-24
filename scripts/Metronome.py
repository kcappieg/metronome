#!/usr/bin/env python3

import copy
import json
import os
from glob import glob
from subprocess import run
from tqdm import tqdm
import pandas as pd
import time
from distlre.distlre import DistLRE, Task, RemoteHost

__author__ = 'Bence Cserna, William Doyle, Kevin C. Gall'

# flags for changing script behavior
ENABLE_SLACK_NOTIFICATION = True
SLACK_CHANNEL = '#experiments'

EXECUTE_REMOTE = True
REMOTE_HOSTS = ['ai' + str(i) + '.cs.unh.edu' for i in
                [1, 2, 3, 4, 6, 8, 10, 11, 12, 13, 14, 15]]
LOCAL_THREADS = 14  # Number of local threads if not executing on remote servers
LOCAL_MACHINE_NAME = 'byodoin.cs.unh.edu'

time_limit_seconds = 60 * 60  # time limit for experiments - 1 hour
results_file_name = 'results/grd-3-23-21_optimal.json'
test_run = True


def generate_base_configuration():
    # refactored for Active Goal Recognition

    # required configuration parameters
    algorithms_to_run = ['NAIVE_OPTIMAL_AGRD']
    lookahead_type = ['DYNAMIC']
    time_limit = [time_limit_seconds * 1_000_000_000]  # as nanoseconds
    # action_durations = [1]  # Use this for A*

    termination_types = ['TIME']
    # expansion_limit = [100000000]

    base_configuration = dict()
    base_configuration['algorithmName'] = algorithms_to_run
    # base_configuration['expansionLimit'] = expansion_limit
    base_configuration['lookaheadType'] = lookahead_type
    # base_configuration['actionDuration'] = action_durations
    base_configuration['terminationType'] = termination_types
    base_configuration['timeLimit'] = time_limit
    base_configuration['commitmentStrategy'] = ['SINGLE']
    # base_configuration['commitmentStrategy'] = ['MULTIPLE']
    base_configuration['heuristicMultiplier'] = [1.0]
    base_configuration['weight'] = [1.0]
    # base_configuration['terminationTimeEpsilon'] = [5_000_000]  # 5ms

    compiled_configurations = [{}]

    for key, value in base_configuration.items():
        compiled_configurations = cartesian_product(compiled_configurations,
                                                    key, value)

    # Algorithm specific configurations

    # Weights included as example for how to configure for specific algorithms
    weights = [1.0]
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'weight', weights,
                                                [['algorithmName',
                                                  'WEIGHTED_A_STAR']])

    # AGRD configurations
    subject_algorithms = ['ALL_PLANS_DYNAMIC']
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'subjectAlgorithm', subject_algorithms,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD']])

    intervention_costs = [1]
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'interventionCost', intervention_costs,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD']])

    iterative_widening = [
        False
        # True
    ]
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'grdIterativeWidening', iterative_widening,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD']])

    optimal_action_durations = [1]
    iter_widening_action_durations = [1_000_000, 10_000_000, 100_000_000, 1_000_000_000]  # ns
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'actionDuration', optimal_action_durations,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD'],
                                                 ['grdIterativeWidening', False]])
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'actionDuration', iter_widening_action_durations,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD'],
                                                 ['grdIterativeWidening', True]])

    # 10 seeds. Should be enough...
    seeds = [4002326368, 2758212009, 3710981193, 2660714033, 3685384885,
             3949298516, 3064535188, 3829612148, 2438865198, 1400649160]
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'seed', seeds,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD']])

    max_depth = [1000]
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'maxDepth', max_depth,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD']])

    return compiled_configurations


def generate_agrd_configs():
    base_configs = generate_base_configuration()

    paths_by_goal_count = {
        2: [],
        3: [],
        4: []
    }
    priors_by_goal_count = {
        2: [0.5, 0.5],
        3: [0.3333333, 0.3333333, 0.3333333],
        4: [0.25, 0.25, 0.25, 0.25]
    }

    # Build all domain paths
    # Using glob, get all files in relevant directories
    slice_from_beginning = len('./resources/input/')

    for goal_count in range(2,5):
        # uniform
        paths_by_goal_count[goal_count] += [
            instance_path[slice_from_beginning:]
            for instance_path in glob(f'./resources/input/vacuum/grd/uniform/{goal_count}goal/*.vw')
        ]

        # rooms
        paths_by_goal_count[goal_count] += [
            instance_path[slice_from_beginning:]
            for instance_path in glob(f'./resources/input/vacuum/grd/rooms/{goal_count}goal/*.vw')
        ]

    configurations = []
    gw_configs = cartesian_product(base_configs, 'domainName',
                                   ['GRID_WORLD'])
    for goal_count in range(2, 5):
        configs_with_priors = cartesian_product(gw_configs, 'goalPriors',
                                                [priors_by_goal_count[goal_count]])
        # config for each goal
        configs_with_priors = cartesian_product(configs_with_priors, 'subjectGoal',
                                                list(range(goal_count)))

        configurations += cartesian_product(configs_with_priors, 'domainPath',
                                            paths_by_goal_count[goal_count])

    # reset
    # TODO: split into 2 functions here
    paths_by_goal_count = {
        2: [],
        3: [],
        4: []
    }

    # logistics
    for goal_count in range(2, 5):
        paths_by_goal_count[goal_count] += [
            instance_path[slice_from_beginning:]
            for instance_path in glob(f'./resources/input/logistics/{goal_count}goal/*.logistics')
        ]

    log_configs = cartesian_product(base_configs, 'domainName', ['LOGISTICS'])
    for goal_count in range(2, 5):
        configs_with_priors = cartesian_product(log_configs, 'goalPriors',
                                                [priors_by_goal_count[goal_count]])
        # config for each goal
        configs_with_priors = cartesian_product(configs_with_priors, 'subjectGoal',
                                                list(range(goal_count)))
        configurations += cartesian_product(configs_with_priors, 'domainPath',
                                            paths_by_goal_count[goal_count])

    return configurations


def config_from_file(filename):
    with open(filename) as file:
        config = json.load(file)
        if type(config) is not list:
            config = [config]

        return config


def cartesian_product(base, key, values, filters=None):
    new_base = []
    if filters is None:
        for item in base:
            for value in values:
                new_configuration = copy.deepcopy(item)
                new_configuration[key] = value
                new_base.append(new_configuration)
    else:
        for item in base:
            if all(filter_key in item and item[filter_key] == filter_value for
                   filter_key, filter_value in filters):
                new_base.extend(cartesian_product([item], key, values))
            else:
                new_base.append(item)

    return new_base


def distributed_execution(configurations, resource_dir=None):
    from slack_notification import start_experiment_notification, \
        end_experiment_notification

    if EXECUTE_REMOTE:
        executor = create_remote_distlre_executor()
    else:
        executor = create_local_distlre_executor(LOCAL_THREADS)

    futures = []
    progress_bar = tqdm(total=len(configurations), smoothing=0.1)
    tqdm.monitor_interval = 0

    cwd = os.getcwd()

    for configuration in configurations:
        executable = '/'.join([cwd, 'build/release/Metronome'])
        resources = resource_dir
        if (resources is None):
            resources = '/'.join([cwd, 'resources/input/'])

        json_configuration = f'{json.dumps(configuration)}\n\n'

        metadata = str(json_configuration)
        command = ' '.join([executable, resources, metadata])

        # giving an additional 60 seconds to task. We want the cpp code to time out so we can get some info
        # about the run
        task = Task(command=command, meta=None, time_limit=time_limit_seconds + 60, memory_limit=10)
        task.input = json_configuration.encode()

        # print(task.command)
        # print(task.input)

        future = executor.submit(task)
        future.add_done_callback(lambda _: progress_bar.update())
        future.configuration = configuration

        futures.append(future)

    if ENABLE_SLACK_NOTIFICATION:
        if EXECUTE_REMOTE:
            machine = REMOTE_HOSTS
        else:
            machine = LOCAL_MACHINE_NAME
        start_experiment_notification(experiment_count=len(configurations), machine=machine, channel=SLACK_CHANNEL)
    print('Experiments started')
    executor.execute_tasks()

    executor.wait()
    progress_bar.close()

    print('Experiments finished')
    if ENABLE_SLACK_NOTIFICATION:
        end_experiment_notification(channel=SLACK_CHANNEL)

    results = construct_results(futures)

    return results


def construct_results(futures):
    results = []
    for future in futures:
        exception = future.exception()
        if exception:
            results.append({
                'configuration': future.configuration,
                'success': False,
                'errorMessage': 'exception ::' + str(exception)
            })
            continue

        result = future.result()
        # print(f'output: {result.output}')

        # print('Output:')
        # print('\n'.join(raw_output))
        if len(result.error) > 0:
            print('Error:')
            print(result.error)

        if result.output is None:
            print('No output error')
            print(future.configuration)

        # allow exception for now... To stop the process
        raw_output = result.output.splitlines()
        if '#' not in raw_output:
            results.append({
                'configuration': future.configuration,
                'success': False,
                'errorMessage': 'exception :: output not found :: ' +
                                str(raw_output)
            })
            continue

        result_offset = raw_output.index('#') + 1

        if result_offset >= len(raw_output):
            results.append({
                'configuration': future.configuration,
                'success': False,
                'errorMessage': 'exception :: incomplete output :: ' +
                                str(raw_output)
            })
            continue

        # print(raw_output[result_offset])
        output = json.loads(raw_output[result_offset])
        results.append(output)
    return results


def create_local_distlre_executor(local_threads):
    executor = DistLRE(local_threads=local_threads)

    return executor


def create_remote_distlre_executor(local_threads=None):
    print('\nExecuting configurations on the following ai servers: ')
    print(REMOTE_HOSTS)

    # I would recommend setting up public key auth for your ssh
    # import getpass
    # password = getpass.getpass("Password to connect to [ai.cs.unh.edu]")
    password = None
    remote_hosts = [RemoteHost(host, 'echo 1', port=22, password=password) for host in
                    REMOTE_HOSTS]

    if local_threads:
        return DistLRE(remote_hosts=remote_hosts, local_threads=local_threads)

    executor = DistLRE(remote_hosts=remote_hosts)
    return executor


def read_results_from_file(file_name):
    if file_name.endswith('.gz'):
        with gzip.open("input.json.gz", "rb") as file:
            return json.loads(file.read().decode("utf-8"))

    with open(file_name) as file:
        return json.load(file)


def inplace_merge_experiments(old_results, new_results):
    for new_result in new_results:
        replaced = False
        for i, old_result in enumerate(old_results):
            if old_result['configuration'] == new_result['configuration']:
                old_results[i] = new_result
                replaced = True
                break

        if not replaced:
            old_results.append(new_result)


def extract_configurations_from_failed_results(results):
    return [result['configuration'] for result in results if
            not result['success']]


def build_metronome():
    if not os.path.exists('build/release'):
        os.makedirs('build/release')

    os.chdir('build/release')

    return_code = run(
        ['cmake',
         '-DCMAKE_BUILD_TYPE=Release',
         '../..']).returncode

    if return_code != 0:
        os.chdir('../..')
        return False

    return_code = run(
        ['cmake --build . --target Metronome -- -j4'],
        # ['cmake --build . --target Metronome --clean-first -- -j4'],
        shell=True).returncode

    os.chdir('../..')
    return return_code == 0


def print_summary(results_json):
    results = pd.read_json(json.dumps(results_json))
    print('Successful: {}/{}'.format(results.success.sum(), len(results_json)))


def save_results(results_json, file_name):
    if not os.path.exists('results'):
        os.makedirs('results')

    with open(file_name, 'w') as outfile:
        json.dump(results_json, outfile)
    print(f'Results saved to {file_name}')


def label_algorithms(configurations):
    pass


def main():
    print(os.getcwd())
    os.chdir('..')

    #     recycle = True
    recycle = False

    if not build_metronome():
        raise Exception('Build failed.')
    print('Build complete!') 

    if recycle:
        # Load previous configurations
        old_results = read_results_from_file(file_name)
        configurations = extract_configurations_from_failed_results(old_results)

        for configuration in configurations:
            configuration['timeLimit'] = 5 * 60 * 1000000000
    else:
        # Generate new domain configurations
        configurations = generate_agrd_configs()
        # configurations = config_from_file('resources/configuration/grd.json')

        label_algorithms(configurations)
        if test_run:
            configurations = configurations[:1]  # debug - keep only one config

    print('{} configurations has been generated '.format(len(configurations)))
    # print(configurations)

    start_time = time.perf_counter()
    results = distributed_execution(configurations)
    end_time = time.perf_counter()

    print(f"Experiment time: {end_time - start_time}s")

    if recycle:
        inplace_merge_experiments(old_results, results)
        results = old_results

    for result in results:
        result.pop('actions', None)
        result.pop('systemProperties', None)

    save_results(results, results_file_name)
    print_summary(results)

    print('{} results has been received.'.format(len(results)))


if __name__ == '__main__':
    main()
