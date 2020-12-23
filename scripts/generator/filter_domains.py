import os
import sys
from shutil import copyfile


def generate_filter_configs(domain_paths, domain_type):
    config_list = []

    for domain_path in domain_paths:
        domain_path_tmp = domain_path[1:] if domain_path[0] == '.' else domain_path

        config = dict()
        config['algorithmName'] = 'A_STAR'
        config['actionDuration'] = 1
        config['domainName'] = domain_type
        config['terminationType'] = 'EXPANSION'
        config['lookaheadType'] = 'DYNAMIC'
        config['commitmentStrategy'] = 'SINGLE'
        config['heuristicMultiplier'] = 1.0
        # strip leading . char from domain
        config['domainPath'] = domain_path_tmp

        config_list.append(config)

    return config_list


def filter_domains(generated_domain_paths, base_domain_name, domain_type='GRID_WORLD', out_path='./gridworld',
                   write=True):
    this_cwd = os.getcwd()

    success_index = 0

    filtered_dir = os.path.join(out_path, 'filtered')
    if not os.path.exists(filtered_dir):
        os.makedirs(filtered_dir)
    configs = generate_filter_configs(generated_domain_paths, domain_type)

    sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    from Metronome import distributed_execution

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
                new_file_name = os.path.join(filtered_dir, base_domain_name + str(success_index) + '.vw')
                print(f'Outputting to {new_file_name}')
                copyfile('.' + result['configuration']['domainPath'], new_file_name)

            success_index += 1
        else:
            print(result['errorMessage'])
            print(f'Domain {result["configuration"]["domainPath"]} was not successfully solved')

    return success_domains