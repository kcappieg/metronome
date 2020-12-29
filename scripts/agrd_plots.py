
import gzip
import json
from operator import itemgetter
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from pandas import DataFrame
import seaborn as sns
import statsmodels.stats.api as sms
from matplotlib.backends.backend_pdf import PdfPages

from scripts.metronome_plot import construct_data_frame, read_data, remove_unused_columns, set_rc, add_row, alg_map


__author__ = "Kevin C. Gall"


def main(paths, configs):
    set_rc()
    idx = 0
    for config in configs:
        title, file_name, domain_filter = itemgetter('title', 'file_name', 'domain_filter')(config)
        data = prepare_data(paths, domain_filter)
        plot_runtime(data, title, file_name)


def prepare_data(paths, domain_filter):
    results = []
    for path_name in paths:
        results += read_data(path_name)

    data = construct_data_frame(results)
    if domain_filter is not None:
        data = data[data.domainPath.str.contains(domain_filter, regex=False)]

    # get rid of timeouts
    timeouts = data[~data.success]
    data = data[data.success]
    print(len(timeouts), 'Failed instances')
    remove_unused_columns(data, [
        'actionDuration', 'actionExecutionTime', 'commitmentStrategy', 'errorMessage',
        'goalAchievementTime', 'heuristicMultiplier', 'identityActions', 'idleIterationCount',
        'idlePlanningTime', 'normalizedGoalAchievementTime', 'weight'
    ])

    add_depth_upper_bound(data)
    print(f'Completed instances: {len(data)}')
    # remove depth=0. Could happen if instance starts in goal configuration
    # such instances should be filtered out of experiments... but bandaid here instead
    data = data[data.depthUpperBound > 0]
    print(f'Valid completed instances: {len(data)}')

    return data


def add_depth_upper_bound(data):
    depth_bound_values = []
    for idx, row in data.iterrows():
        most = 0
        second_most = 0
        idx = 0
        while f'Goal_{idx}' in row:
            cost = row[f'Goal_{idx}']
            if cost > most:
                second_most = most
                most = cost
            elif cost > second_most:
                second_most = cost

            idx += 1

        depth_bound_values.append(second_most)

    data['depthUpperBound'] = depth_bound_values


def plot_runtime(data, title, file_name):
    plot_log = True

    results = DataFrame(
        columns="depthUpperBound firstIterationRuntime algorithmName lbound rbound numInstances".split()
    )
    # rescale runtime to ms
    data['firstIterationRuntime'] = data['firstIterationRuntime'] / 1000000

    if plot_log:
        data['firstIterationRuntime'] = np.log(data['firstIterationRuntime'])

    # average runtime for all instances that share a depth bound
    for fields, depth_group in data.groupby(['algorithmName', 'depthUpperBound']):
        if len(depth_group) == 1:
            # no way to compute confidence interval
            continue

        alg_name = fields[0]
        if alg_name in alg_map:
            alg_name = alg_map[alg_name]

        # confidence
        mean_runtime = depth_group['firstIterationRuntime'].mean()
        runtime_list = list(depth_group['firstIterationRuntime'])
        bound = sms.DescrStatsW(runtime_list).zconfint_mean()
        results = add_row(results,
                          [fields[1], mean_runtime, alg_name,
                           abs(mean_runtime - bound[0]),
                           abs(mean_runtime - bound[1]),
                           len(depth_group)])

    errors = []
    for alg, alg_group in results.groupby('algorithmName'):
        errors.append([alg_group['lbound'].values, alg_group['rbound'].values])

    pivot = results.pivot(index="depthUpperBound", columns="algorithmName",
                       values="firstIterationRuntime")
    pivot = pivot[~pivot.index.duplicated(keep='first')]

    palette = sns.color_palette(n_colors=10)
    plot = pivot.plot(color=palette, title=title, legend=True, yerr=errors,
                      ecolor='black', elinewidth=1,
                      capsize=4, capthick=1)

    if plot_log:
        num_levels_below_zero = 1
        num_levels_zero_above = 8

        ticks = [1 / (10 ** num) for num in range(num_levels_below_zero, 0, -1)]
        ticks += [10 ** num for num in range(num_levels_zero_above)]

        plot.set_yticks(np.log(ticks))
        plot.set_yticklabels([num if num < 10000 else f'{num:.1g}' for num in ticks])

    instances_pivot = results.pivot(index="depthUpperBound", columns="algorithmName",
                                    values="numInstances")
    # plot.plot(instances_pivot.index, instances_pivot.values, linestyle='--', color='r')
    print(instances_pivot)

    plot.set_xlabel('Depth Upper Bound')
    plot.set_ylabel('Runtime to Exhaustively Explore Tree (ms)')

    pdf = PdfPages("../results/plots/" + file_name + ".pdf")
    plt.savefig(pdf, format='pdf')
    pdf.close()
    # plt.show()


if __name__ == "__main__":
    main(['../results/grd-12-28-20.json'],
         [
             {
                 'title': 'Optimal AGRD Complexity - Uniform Gridworld',
                 'file_name': 'uniform_agrd_runtime',
                 'domain_filter': 'grd/uniform'
             },
             {
                 'title': 'Optimal AGRD Complexity - Rooms Gridworld',
                 'file_name': 'rooms_agrd_runtime',
                 'domain_filter': 'grd/rooms'
             },
             {
                 'title': 'Optimal AGRD Complexity - Logistics',
                 'file_name': 'logistics_agrd_runtime',
                 'domain_filter': 'grd/logistics'
             },
             {
                 'title': 'Optimal AGRD Complexity',
                 'file_name': 'all_agrd_runtime',
                 'domain_filter': None
             }
         ])
