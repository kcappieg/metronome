#!/usr/bin/env python3

import os
import sys
from shutil import copyfile, move
import argparse
from glob import glob

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from Metronome import distributed_execution


def generate_astar_configs(domain_paths, domain_type):
    config_list = []

    for domain_path in domain_paths:
        # strip leading . char from domain
        domain_path_tmp = domain_path[1:] if domain_path[0] == '.' else domain_path

        config = dict()
        config['algorithmName'] = 'A_STAR'
        config['actionDuration'] = 1
        config['domainName'] = domain_type
        config['terminationType'] = 'EXPANSION'
        config['lookaheadType'] = 'DYNAMIC'
        config['commitmentStrategy'] = 'SINGLE'
        config['heuristicMultiplier'] = 1.0
        config['domainPath'] = domain_path_tmp

        config_list.append(config)

    return config_list


def generate_agrd_configs(domain_paths, domain_type, goals):
    config_list = []

    for domain_path in domain_paths:
        # strip leading . char from domain
        domain_path_tmp = domain_path[1:] if domain_path[0] == '.' else domain_path

        config = dict()
        config['algorithmName'] = 'NAIVE_OPTIMAL_AGRD'
        config['actionDuration'] = 1
        config['interventionCost'] = 1
        config['domainName'] = domain_type
        config['terminationType'] = 'EXPANSION'
        config['subjectAlgorithm'] = 'NAIVE_DYNAMIC'
        config['timeLimit'] = 300_000_000_000  # 300 second (5 min) timeout
        config['maxDepth'] = 1000
        config['goalPriors'] = [1 / goals for _ in range(goals)]
        config['subjectGoal'] = 0
        config['domainPath'] = domain_path_tmp

        config_list.append(config)

    return config_list


def filter_domains(generated_domain_paths, base_domain_name, domain_type='GRID_WORLD', domain_ext='.vw',
                   out_path='./filtered', write=True):
    this_cwd = os.getcwd()

    success_index = 0

    if not os.path.exists(out_path):
        os.makedirs(out_path)
    configs = generate_astar_configs(generated_domain_paths, domain_type)

    print('Begin filtering of generated domains')

    os.chdir('../..')
    results = distributed_execution(configs, this_cwd)
    os.chdir(this_cwd)

    success_domains = []
    for result in results:
        if (result['success']):
            print(f'Domain {result["configuration"]["domainPath"]} is solvable')
            success_domains.append(result["configuration"]["domainPath"])

            if write:
                new_file_name = os.path.join(out_path, base_domain_name + str(success_index) + domain_ext)
                print(f'Outputting to {new_file_name}')
                move('.' + result['configuration']['domainPath'], new_file_name)

            success_index += 1
        else:
            print(result['errorMessage'])
            print(f'Domain {result["configuration"]["domainPath"]} was not successfully solved')

    return success_domains


def get_with_default(d, key, default_value=None, default_producer=None):
    if key not in d:
        d[key] = default_value if default_producer is None else default_producer()

    return d[key]


def get_with_default_list(d, key):
    return get_with_default(d, key, default_value=[])


def get_with_default_dict(d, key):
    return get_with_default(d, key, default_value=dict())


def get_depth_upper_bound(result):
    most = 0
    second_most = 0
    idx = 0
    while f'Goal_{idx}' in result:
        cost = result[f'Goal_{idx}']
        if cost > most:
            second_most = most
            most = cost
        elif cost > second_most:
            second_most = cost

        idx += 1
    return second_most


def filter_agrd_chunk(config, chunk_instances, inactive_out_dir, followup_out_dir):
    this_cwd = os.getcwd()
    base_domain_name = config['base_domain_name']
    domain_ext = config['domain_ext']

    path_to_instance = {
        os.path.join(
            config['source_dir'],
            filename
        ): filename
        for filename in chunk_instances
    }
    configs = generate_agrd_configs(path_to_instance.keys(), config['domain_type'], config['num_goals'])

    os.chdir('../..')
    results = distributed_execution(configs, this_cwd)
    os.chdir(this_cwd)

    successes_by_depth_bound = dict()
    timeouts_by_depth_bound = dict()
    for result in results:
        result['depthUpperBound'] = get_depth_upper_bound(result)
        instance_path = result["configuration"]["domainPath"]
        if instance_path[0] != '.':
            instance_path = '.' + instance_path

        instance_filename = path_to_instance[instance_path]

        if result['success'] and result.get('observerIsActive', 0) > 0:
            print(f'Observer was active in domain {instance_path}')
            get_with_default_list(successes_by_depth_bound, result['depthUpperBound'])\
                .append((instance_path, instance_filename, base_domain_name, domain_ext))
        else:
            if result['success']:
                print(f'Observer was inactive in domain {instance_path}')
                move(instance_path, os.path.join(inactive_out_dir, instance_filename))
            else:
                err_msg = result["errorMessage"]
                print(f'Failed to solve domain {instance_path} with error {err_msg}')

                lower_err = err_msg.lower()
                if 'timeout' in lower_err:
                    get_with_default_list(timeouts_by_depth_bound, result['depthUpperBound'])\
                        .append((instance_path, instance_filename, base_domain_name, domain_ext))
                elif 'dead end' in lower_err or 'subject transitioned' in lower_err:
                    # follow up on instances that fail for reasons that shouldn't happen...
                    move(instance_path, os.path.join(followup_out_dir, instance_filename))
                else:
                    move(instance_path, os.path.join(inactive_out_dir, instance_filename))

    return successes_by_depth_bound, timeouts_by_depth_bound


def move_agrd_filter_results(successes_info_by_depth_bound, timeouts_info_by_depth_bound):
    """Moves successes to new directory, but only if all instances
    at the relevant depth bound succeeded"""
    # loop through timeouts first to purge successes dict
    meta_files_by_out = dict()
    for depth_bound, timeout_info in timeouts_info_by_depth_bound.items():
        for out_dir, timeout_list in timeout_info.items():
            print(f'Moving timeout instances at depth bound {depth_bound} for out dir {out_dir}')

            timeout_dir = os.path.join(out_dir, 'timeout')
            meta_file = get_with_default(
                meta_files_by_out, out_dir,
                default_producer=lambda: open(os.path.join(out_dir, 'stats.log'), 'w'))

            success_info = get_with_default_dict(successes_info_by_depth_bound, depth_bound)
            successes_list = get_with_default_list(success_info, out_dir)

            meta_file.write(f'Depth Bound {depth_bound}: '
                            f'{len(successes_list)} successes, {len(timeout_list)} timeouts\n')
            for instance_path, instance_filename, _, _ in timeout_list + successes_list:
                move(instance_path, os.path.join(timeout_dir, instance_filename))

            success_info[out_dir] = []  # wipe the list so we don't write to success dir later

    for file in meta_files_by_out.values():
        file.write('\n=====================================\n\n')

    success_indices = {}
    for depth_bound, success_info in successes_info_by_depth_bound.items():
        for out_dir, successes_list in success_info.items():
            if len(successes_list) == 0:
                continue

            print(f'Moving successful instances at depth bound {depth_bound} for out dir {out_dir}')

            meta_file = get_with_default(
                meta_files_by_out, out_dir,
                default_producer=lambda: open(os.path.join(out_dir, 'stats.log'), 'w'))

            meta_file.write(f'Depth Bound {depth_bound}: {len(successes_list)} successes\n')

            for instance_path, _, base_domain_name, domain_ext in successes_list:
                prefix = os.path.join(out_dir, base_domain_name)
                new_file_path = prefix + str(get_with_default(success_indices, prefix, 0)) + domain_ext
                success_indices[prefix] += 1

                move(instance_path, new_file_path)

    for file in meta_files_by_out.values():
        file.close()


def filter_active_observer(domain_configs, chunk_size=1000):
    """Filter to only those where the observer is active.
    Dict schema:
        source_dir: str of the source directory
        base_domain_name: str prefix for all instance filenames
        num_instances: number of instances being filtered with this config
        num_goals: number of goals in each instance (must be same across all instances)
        domain_type: 'GRID_WORLD', 'LOGISTICS', etc
        domain_ext: '.vw', '.logistics', etc
        out_dir: str of the output directory
    """
    successes_info_by_depth_bound = dict()
    timeouts_info_by_depth_bound = dict()
    for config in domain_configs:
        base_domain_name = config['base_domain_name']
        domain_ext = config['domain_ext']
        out_dir = config['out_dir']
        src_dir = config['source_dir']
        if src_dir[-1] != '/':
            src_dir += '/'

        print(f'Filtering {base_domain_name} instances')
        timeout_out_dir = os.path.join(out_dir, 'timeout')
        if not os.path.exists(timeout_out_dir):
            os.makedirs(timeout_out_dir)

        inactive_out_dir = os.path.join(out_dir, 'failed')
        if not os.path.exists(inactive_out_dir):
            os.makedirs(inactive_out_dir)

        followup_out_dir = os.path.join(out_dir, 'follow-up')
        if not os.path.exists(followup_out_dir):
            os.makedirs(followup_out_dir)

        domain_instance_filenames = [
            filepath[len(src_dir):]
            for filepath in glob(src_dir + base_domain_name + '*' + domain_ext)
        ]

        idx = 0
        while len(domain_instance_filenames) > idx:
            # new_file_path = os.path.join(active_out_dir, base_domain_name + str(success_index) + domain_ext)
            chunk_instances = domain_instance_filenames[idx:idx + chunk_size]

            print(f'Begin filtering {base_domain_name} {idx} through '
                  f'{min(idx + chunk_size - 1, len(domain_instance_filenames) - 1)}')

            tmp_successes, tmp_failures = filter_agrd_chunk(config, chunk_instances, inactive_out_dir, followup_out_dir)
            for key, value in tmp_successes.items():
                all_success_info = get_with_default_dict(successes_info_by_depth_bound, key)
                group_success_list = get_with_default_list(all_success_info, out_dir)
                group_success_list += value
            for key, value in tmp_failures.items():
                all_failure_info = get_with_default_dict(timeouts_info_by_depth_bound, key)
                group_failure_list = get_with_default_list(all_failure_info, out_dir)
                group_failure_list += value

            idx += chunk_size

    move_agrd_filter_results(successes_info_by_depth_bound, timeouts_info_by_depth_bound)


def run_filter_observer(args):
    domain_identifier = args.domain_identifier

    configs = []
    if domain_identifier == 'uniform':
        for size in range(7, 11):
            base_domain_name = f'uniform{size}_{size}-'
            for goals in range(2, 5):
                dir_name = f'./gridworld/{goals}goal/filtered'
                num_instances = len(glob(os.path.join(dir_name, base_domain_name) + '*'))

                configs.append({
                    'source_dir': dir_name,
                    'base_domain_name': base_domain_name,
                    'num_instances': num_instances,
                    'num_goals': goals,
                    'domain_type': 'GRID_WORLD',
                    'domain_ext': '.vw',
                    'out_dir': f'./agrd/uniform/{goals}goal'
                })
    elif domain_identifier == 'rooms':
        for idx in range(10):
            base_domain_name = f'64room_tiny_00{idx}-scn'
            for goals in range(2, 5):
                dir_name = f'./gridmap/{goals}goal/filtered'
                num_instances = len(glob(os.path.join(dir_name, base_domain_name) + '*'))

                configs.append({
                    'source_dir': dir_name,
                    'base_domain_name': base_domain_name,
                    'num_instances': num_instances,
                    'num_goals': goals,
                    'domain_type': 'GRID_WORLD',
                    'domain_ext': '.vw',
                    'out_dir': f'./agrd/rooms/{goals}goal'
                })
    elif domain_identifier == 'logistics':
        pass
        for locs in range(7, 12):
            for goals in range(2, 5):
                base_domain_name = f'geometric_0.4dist_{goals}goal_{locs}loc_3pkg_1trk_'
                dir_name = f'./logistics/{goals}goal'
                num_instances = len(glob(os.path.join(dir_name, base_domain_name) + '*'))

                configs.append({
                    'source_dir': dir_name,
                    'base_domain_name': base_domain_name,
                    'num_instances': num_instances,
                    'num_goals': goals,
                    'domain_type': 'LOGISTICS',
                    'domain_ext': '.logistics',
                    'out_dir': f'./agrd/logistics/{goals}goal'
                })
    else:
        raise Exception(f'Unknown domain identifier: {domain_identifier}')

    # log_config = {
    #     'source_dir': './logistics',
    #     'base_domain_name': 'geometric_0.4dist_3goal_15loc_3pkg_1trk_',
    #     'num_instances': 2,
    #     'num_goals': 3,
    #     'domain_type': 'LOGISTICS',
    #     'domain_ext': '.logistics',
    #     'out_dir': './test/logistics'
    # }

    filter_active_observer(configs)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Quick and dirty CLI for filtering AGRD instances by only '
                    'those where the observer can actually do something. '
                    'To use, edit the file')
    # AS OF 1/6/20, valid options are 'logistics', 'rooms', 'uniform'
    parser.add_argument('domain_identifier', type=str,
                        help='String identifier for your set of domains.')

    run_filter_observer(parser.parse_args())
