#!/usr/bin/env python3

import copy
import json
import os
from subprocess import run
from tqdm import tqdm
import pandas as pd
import time
from distlre.distlre import DistLRE, Task, RemoteHost

__author__ = 'Bence Cserna, William Doyle, Kevin C. Gall'


def generate_base_configuration():
    # refactored for Active Goal Recognition

    # required configuration parameters
    algorithms_to_run = ['NAIVE_OPTIMAL_AGRD']
    lookahead_type = ['DYNAMIC']
    time_limit = [10 * 60 * 1000000000] # min as nanoseconds
    action_durations = [1]  # Use this for A*

    termination_types = ['EXPANSION']
    # expansion_limit = [100000000]

    base_configuration = dict()
    base_configuration['algorithmName'] = algorithms_to_run
    # base_configuration['expansionLimit'] = expansion_limit
    base_configuration['lookaheadType'] = lookahead_type
    base_configuration['actionDuration'] = action_durations
    base_configuration['terminationType'] = termination_types
    base_configuration['timeLimit'] = time_limit
    base_configuration['commitmentStrategy'] = ['SINGLE']
    base_configuration['commitmentStrategy'] = ['MULTIPLE']
    base_configuration['heuristicMultiplier'] = [1.0]
    base_configuration['weight'] = [1.0]
    # base_configuration['terminationTimeEpsilon'] = [5000000]  # 5ms

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
    subject_algorithms = ['NAIVE_DYNAMIC']
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'subjectAlgorithm', subject_algorithms,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD']])

    intervention_costs = [1]
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'interventionCost', intervention_costs,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD']])

    max_depth = [1000]
    compiled_configurations = cartesian_product(compiled_configurations,
                                                'maxDepth', max_depth,
                                                [['algorithmName',
                                                  'NAIVE_OPTIMAL_AGRD']])

    return compiled_configurations


def generate_grid_world():
    configurations = generate_base_configuration()

    domain_paths = []

    # Build all domain paths

    # uniform 3-goals
    uniform_3goal_base_path = 'input/vacuum/grd/uniform_3goals/uniform'
    uniform_3goal_paths = []

    for size in range(7, 12):
        for scenario_num in range(0, 20):
            uniform_3goal_paths.append(f'{uniform_3goal_base_path}{size}_{size}-{scenario_num}.vw')

    domain_paths.extend(uniform_3goal_paths)

    configurations = cartesian_product(configurations, 'domainName',
                                       [
                                           'GRID_WORLD'
                                       ])
    configurations = cartesian_product(configurations, 'domainPath',
                                       domain_paths)

    # goal priors for all - 3 goals as list, so must make it a list of list
    priors = [[0.33, 0.33, 0.34]]
    configurations = cartesian_product(configurations, 'goalPriors',
                                       priors)

    return configurations


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
    # from slack_notification import start_experiment_notification, \
    #     end_experiment_notification

    # executor = create_remote_distlre_executor()
    executor = create_local_distlre_executor(6)

    futures = []
    progress_bar = tqdm(total=len(configurations), smoothing=0.1)
    tqdm.monitor_interval = 0

    cwd = os.getcwd()

    for configuration in configurations:
        executable = '/'.join([cwd, 'build/release/Metronome'])
        resources = resource_dir
        if (resources is None):
            resources = '/'.join([cwd, 'resources/input/vacuum/'])

        json_configuration = f'{json.dumps(configuration)}\n\n'

        metadata = str(json_configuration)
        command = ' '.join([executable, resources, metadata])

        task = Task(command=command, meta=None, time_limit=90, memory_limit=10)
        task.input = json_configuration.encode()

        # print(task.command)
        # print(task.input)

        future = executor.submit(task)
        future.add_done_callback(lambda _: progress_bar.update())
        future.configuration = configuration

        futures.append(future)

    # start_experiment_notification(experiment_count=len(configurations))
    print('Experiments started')
    executor.execute_tasks()

    executor.wait()
    progress_bar.close()

    print('Experiments finished')
    # end_experiment_notification()

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

        raw_output = result.output.splitlines()
        # print('Output:')
        # print('\n'.join(raw_output))
        if len(result.error) > 0:
            print('Error:')
            print(result.error)
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
    from slack_notification import start_experiment_notification, \
        end_experiment_notification

    import getpass
    HOSTS = ['ai' + str(i) + '.cs.unh.edu' for i in
             [1, 2, 3, 4, 5, 6, 8, 9, 11, 12, 13, 14, 15]]
    print('\nExecuting configurations on the following ai servers: ')
    print(HOSTS)

    # I would recommend setting up public key auth for your ssh
    # password = getpass.getpass("Password to connect to [ai.cs.unh.edu]")
    password = None
    remote_hosts = [RemoteHost(host, port=22, password=password) for host in
                    HOSTS]

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

    file_name = 'results/grd-test.json'

    if recycle:
        # Load previous configurations
        old_results = read_results_from_file(file_name)
        configurations = extract_configurations_from_failed_results(old_results)
        
        for configuration in configurations:
            configuration['timeLimit'] = 5 * 90 * 1000 * 1000000
    else:
        # Generate new domain configurations
        configurations = generate_grid_world()

        label_algorithms(configurations)
        # configurations = configurations[:1]  # debug - keep only one config

    print('{} configurations has been generated '.format(len(configurations)))
    
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

    save_results(results, 'results/results_temp.json')

    save_results(results, file_name)
    print_summary(results)

    print('{} results has been received.'.format(len(results)))


if __name__ == '__main__':
    main()
