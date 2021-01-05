import os
import sys
from shutil import copyfile, move


def import_dist_ex():
    sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    from Metronome import distributed_execution
    return distributed_execution


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
        config['timeLimit'] = 30000000000
        config['maxDepth'] = 1000,
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

    distributed_execution = import_dist_ex()

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
                copyfile('.' + result['configuration']['domainPath'], new_file_name)

            success_index += 1
        else:
            print(result['errorMessage'])
            print(f'Domain {result["configuration"]["domainPath"]} was not successfully solved')

    return success_domains


def filter_active_observer(domain_configs, chunk_size=100):
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
    # TODO: Label stats on depth bound for succeeded and failed domains
    distributed_execution = import_dist_ex()
    this_cwd = os.getcwd()

    for config in domain_configs:
        base_domain_name = config['base_domain_name']
        domain_ext = config['domain_ext']

        print(f'Filtering {base_domain_name} instances')

        active_out_dir = os.path.join(config['out_dir'], 'filtered')
        if not os.path.exists(active_out_dir):
            os.makedirs(active_out_dir)

        inactive_out_dir = os.path.join(config['out_dir'], 'failed')
        if not os.path.exists(inactive_out_dir):
            os.makedirs(inactive_out_dir)

        success_index = 0

        domain_instance_filenames = [
            base_domain_name + str(i) + domain_ext
            for i in range(config['num_instances'])
        ]

        idx = 0
        while len(domain_instance_filenames) > idx:
            chunk_instances = domain_instance_filenames[idx:idx + chunk_size]
            path_to_instance = {
                os.path.join(
                    config['source_dir'],
                    filename
                ): filename
                for filename in chunk_instances
            }
            configs = generate_agrd_configs(path_to_instance.keys(), config['domain_type'], config['num_goals'])

            print(f'Begin filtering {base_domain_name} {idx} through '
                  f'{min(idx + chunk_size - 1, len(domain_instance_filenames) - 1)}')

            os.chdir('../..')
            results = distributed_execution(configs, this_cwd)
            os.chdir(this_cwd)

            for result in results:
                instance_path = result["configuration"]["domainPath"]
                if instance_path[0] != '.':
                    instance_path = '.' + instance_path

                instance_filename = path_to_instance[instance_path]

                if result['success'] and result.get('observerIsActive', 0) > 0:
                    print(f'Observer was active in domain {instance_path}')
                    new_file_path = os.path.join(active_out_dir, base_domain_name + str(success_index) + domain_ext)
                    print(f'Outputting to {new_file_path}')

                    move(instance_path, new_file_path)

                    success_index += 1
                else:
                    if result['success']:
                        print(f'Observer was inactive in domain {instance_path}')
                    else:
                        print(f'Failed to solve domain {instance_path} with error {result["errorMessage"]}')

                    move(instance_path, os.path.join(inactive_out_dir, instance_filename))

            idx += chunk_size


def run_filter_observer():
    """Used as scratch pad"""
    log_config = {
        'source_dir': './logistics',
        'base_domain_name': 'geometric_0.4dist_3goal_15loc_3pkg_1trk_',
        'num_instances': 2,
        'num_goals': 3,
        'domain_type': 'LOGISTICS',
        'domain_ext': '.logistics',
        'out_dir': './test/logistics'
    }

    # grid_config = {
    #     'source_dir': './gridworld',
    #     'base_domain_name': 'uniform7_7-',
    #     'num_instances': 2,
    #     'num_goals': 2,
    #     'domain_type': 'GRID_WORLD',
    #     'domain_ext': '.vw',
    #     'out_dir': './test/gridworld'
    # }
    #
    # temp_config = {
    #     'source_dir': './temp',
    #     'base_domain_name': 'test-',
    #     'num_instances': 1,
    #     'num_goals': 2,
    #     'domain_type': 'GRID_WORLD',
    #     'domain_ext': '.vw',
    #     'out_dir': './test/temp'
    # }

    filter_active_observer([log_config])


if __name__ == '__main__':
    run_filter_observer()
