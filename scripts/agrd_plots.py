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

PLOT_LOG = True


def main(paths, configs):
    set_rc()
    idx = 0
    for config in configs:
        title, file_name, domain_filter = itemgetter('title', 'file_name', 'domain_filter')(config)

        print('\n========================================')
        print(f'Domain {title}\n')

        data = prepare_data(paths, domain_filter)
        # plot_optimal_runtime(data, title, file_name)
        # plot_optimal_scatter(data, title, file_name)
        plot_suboptimal_comparison(data, title, file_name)


def prepare_data(paths, domain_filter):
    results = []
    for path_name in paths:
        results += read_data(path_name)

    data = construct_data_frame(results)
    if domain_filter is not None:
        data = data[data.domainPath.str.contains(domain_filter, regex=False)]

    # get rid of timeouts
    errors = data[~data.success]
    data = data[data.success]
    print(len(errors), 'Failed instances')
    error_messages = errors['errorMessage'].unique()
    print(f'Debug here to see {len(error_messages)} unique error messages')
    remove_unused_columns(data, [
        'actionExecutionTime', 'commitmentStrategy', 'errorMessage',
        'goalAchievementTime', 'heuristicMultiplier', 'identityActions', 'idleIterationCount',
        'idlePlanningTime', 'normalizedGoalAchievementTime', 'weight'
    ])

    add_depth_upper_bound(data)
    add_num_goals(data)

    print(f'Completed instances: {len(data)}')
    # remove depth=0. Could happen if instance starts in goal configuration
    # such instances should be filtered out of experiments... but bandaid here instead
    data = data[data.depthUpperBound > 0]
    print(f'Instances where depthUpperBound > 0: {len(data)}')
    # remove depth>6 for grid world b/c no grid world could complete depth >= 6 without timing out
    # some instances.
    data = data[
        (data.domainName == 'LOGISTICS') |
        ((data.domainName != 'LOGISTICS') & (data.depthUpperBound < 6))
    ]

    print(f'Valid completed instances: {len(data)}')

    # rescale runtime to ms
    data['firstIterationRuntime_ms'] = data['firstIterationRuntime'] / 1000000
    data['logRuntime'] = np.log(data['firstIterationRuntime_ms'])

    data['runtime_axis'] = data['logRuntime'] if PLOT_LOG else data['firstIterationRuntime_ms']

    data = extract_quality_metrics(data)
    print(f'Instances with optimal comparison: {len(data)}')

    return data


def extract_quality_metrics(data):
    # first, extract the actual metric
    # This is for fraction of optimal

    # fix cases where goal was never reported
    adjustment_reported_iteration = data[data.goalReportedIteration == 0].pathLength
    data.loc[data.goalReportedIteration == 0, 'goalReportedIteration'] = adjustment_reported_iteration
    data['fractionOfPath'] = data.goalReportedIteration / data.pathLength

    # isolate each scene with the subject's true goal
    data['domainPathWithGoal'] = data.domainPath.map(str) + data.subjectGoal.map(str) + data.seed.map(str)

    # mark all instances that represent optimal AGRD
    data['isOptimal'] = data['grdIterativeWidening'] == False # noqa
    optimal_data = data[data['isOptimal'] == True] # noqa

    # average the optimal results for a given domain.
    # e.g. if 10 seeds for each subject goal, we should have 20 instances for 2-goal scene
    avg_achieved_by_domain = optimal_data.groupby('domainPathWithGoal').fractionOfPath.mean()
    avg_achieved_by_domain.name = 'avgOptimalFractionOfPath'
    data = data.merge(avg_achieved_by_domain, how='inner', on='domainPathWithGoal', copy=False)

    # remove any instances where we don't have an optimal baseline
    data = data[data['avgOptimalFractionOfPath'] > 0]  # should filter out NaNs, which could be for timed out instances

    # deviation from average optimal baseline. Negative means better
    deviation_from_baseline = data['fractionOfPath'] - data['avgOptimalFractionOfPath']
    data['percentDeviationFromBaseline'] = deviation_from_baseline / data['avgOptimalFractionOfPath']

    return data


def add_depth_upper_bound(data):
    depth_bound_values = []
    for _, row in data.iterrows():
        most = 0
        second_most = 0
        idx = 0
        while f'Goal_{idx}' in row:
            cost = row[f'Goal_{idx}'] / row['actionDuration']
            if cost > most:
                second_most = most
                most = cost
            elif cost > second_most:
                second_most = cost

            idx += 1

        depth_bound_values.append(int(second_most))

    data['depthUpperBound'] = depth_bound_values


def add_num_goals(data):
    num_goals_series = [len(priors) for priors in data.goalPriors]
    data['numGoals'] = num_goals_series


def plot_optimal_runtime(data, title, file_name):
    data = data[data['isOptimal'] == True]  # noqa

    results = DataFrame(
        columns="depthUpperBound runtime algorithmName lbound rbound numInstances".split()
    )

    # average runtime for all instances that share a depth bound
    for fields, depth_group in data.groupby(['algorithmName', 'depthUpperBound']):
        if len(depth_group) == 1:
            # no way to compute confidence interval
            continue

        alg_name = fields[0]
        if alg_name in alg_map:
            alg_name = alg_map[alg_name]

        # confidence
        mean_runtime = depth_group['runtime_axis'].mean()
        runtime_list = list(depth_group['runtime_axis'])
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
                          values="runtime")
    pivot = pivot[~pivot.index.duplicated(keep='first')]

    palette = sns.color_palette(n_colors=10)
    # plt_title = 'Optimal AGRD Complexity' + ('' if title is None else ' - ' + title)
    plt_title = title
    plot = pivot.plot(color=palette, title=plt_title, legend=False, yerr=errors,
                      ecolor='black', elinewidth=1,
                      capsize=4, capthick=1,
                      figsize=(3, 3))

    if PLOT_LOG:
        set_log_y_ticks(plot, 5, 0)

    set_x_ticks(plot, data, 'depthUpperBound')

    instances_pivot = results.pivot(index="depthUpperBound", columns="algorithmName",
                                    values="numInstances")
    # plot.plot(instances_pivot.index, instances_pivot.values, linestyle='--', color='r')
    print(instances_pivot)

    plot.set_xlabel('Depth Upper Bound')
    plot.set_ylabel('Runtime to Exhaustively Explore Tree (ms)')

    # plt.subplots_adjust(left=.3, bottom=.3)

    pdf = PdfPages("../results/plots/" + file_name + "_agrd_runtime.pdf")
    plt.savefig(pdf, format='pdf', bbox_inches='tight')
    pdf.close()
    plt.show()


def plot_optimal_scatter(data, title, file_name):
    data = data[data['isOptimal'] == True]  # noqa
    if len(data) == 0:
        return

    # TODO: Bar plot w/ confidence. Depth is category, num goals is category that creates bars
    # y is avg runtime, confidence over that should hopefully overlap
    fig, ax = plt.subplots()

    plt_title = 'Runtime Scatter' if title is None else title

    colors = sns.color_palette('dark', len(data.numGoals.unique()))
    color_map = dict(zip(data.numGoals.unique(), colors))

    results = DataFrame(
        columns="depthUpperBound avg_runtime numGoals instances".split()
    )

    for fields, group in data.groupby(['numGoals', 'depthUpperBound']):
        results = add_row(results, [
            fields[1], group.runtime_axis.mean(), int(fields[0]), len(group)
        ])

    for key, group in results.groupby('numGoals'):
        group.plot.scatter(ax=ax, s=group.instances * 10,  # bubble size
                           alpha=0.5,
                           x='depthUpperBound', y='avg_runtime',
                           label=key, color=color_map[key])

    ax.set_title(plt_title)
    ax.set_xlabel('Depth Upper Bound')
    ax.set_ylabel('Runtime to Exhaustively Explore Tree (ms)')

    ax.legend(title='Goal Count')

    set_x_ticks(ax, data, 'depthUpperBound')

    if PLOT_LOG:
        set_log_y_ticks(ax, 5)

    # TODO: save plot to pdf

    plt.show()


def plot_suboptimal_comparison(data, title, file_name):
    data = data[data['isOptimal'] == False]  # noqa
    if len(data) == 0:
        return

    results = DataFrame(
        columns="timeBound percentDeviationFromBaseline algorithmName lbound rbound".split()
    )

    for fields, depth_group in data.groupby(['depthUpperBound', 'actionDuration']):
        if len(depth_group) == 1:
            # no way to compute confidence interval
            continue

        alg_name = 'IW-AGRD (Depth ' + str(fields[0]) + ')'
        if alg_name in alg_map:
            alg_name = alg_map[alg_name]

        # confidence
        mean_deviation = depth_group['percentDeviationFromBaseline'].mean()
        deviation_list = list(depth_group['percentDeviationFromBaseline'])
        bound = sms.DescrStatsW(deviation_list).zconfint_mean()
        results = add_row(results,
                          [fields[1] / 1e6, mean_deviation, alg_name,
                           abs(mean_deviation - bound[0]),
                           abs(mean_deviation - bound[1])])

    # TODO: plot quality of solution as a function of allowable time
    # Different line (different figure?) for every depth bound
    # Average by time bound, 95% confidence intervals
    # Use 'percentDeviationFromBaseline' as metric

    errors = []
    for alg, alg_group in results.groupby('algorithmName'):
        errors.append([alg_group['lbound'].values, alg_group['rbound'].values])

    pivot = results.pivot(index="timeBound", columns="algorithmName",
                          values="percentDeviationFromBaseline")
    # pivot = pivot[~pivot.index.duplicated(keep='first')]

    palette = sns.color_palette(n_colors=10)
    plt_title = title
    plot = pivot.plot(color=palette, title=plt_title, yerr=errors,
                      ecolor='black', elinewidth=1,
                      capsize=4, capthick=1
                      # , figsize=(3, 3)
                      )
    plot.autoscale(True)
    plot.margins(0.05)

    plot.set_xscale('log')
    plot.set_xlabel('Iteration Time Bounds (ms)')
    plot.set_ylabel('Percent Deviation from Optimal Baseline')

    pdf = PdfPages("../results/plots/" + file_name + "_iterative-widening_results.pdf")
    plt.savefig(pdf, format='pdf', bbox_inches='tight')
    pdf.close()
    plt.show()


def set_log_y_ticks(plot, num_levels_zero_above=5, num_levels_below_zero = 1):
    ticks = [1 / (10 ** num) for num in range(num_levels_below_zero, 0, -1)]
    ticks += [10 ** num for num in range(num_levels_zero_above)]

    plot.set_yticks(np.log(ticks))
    plot.set_yticklabels([num if num < 10000 else f'{num:.1g}' for num in ticks])


def set_x_ticks(plot, data, field_name):
    xticks = [x_val for x_val in range(1, int(data[field_name].max() + 1))]
    plot.set_xticks(xticks)
    plot.set_xlim(xticks[0] - 0.3, xticks[len(xticks) - 1] + 0.3)


if __name__ == "__main__":
    main(['../results/grd-3-16-21_optimal_vs_suboptimal-fraction_of_path.json'],
         [
             {
                 'title': 'Uniform Gridworld',
                 'file_name': 'uniform',
                 'domain_filter': 'grd/uniform'
             },
             {
                 'title': 'Rooms Gridworld',
                 'file_name': 'rooms',
                 'domain_filter': 'grd/rooms'
             },
             {
                 'title': 'Logistics',
                 'file_name': 'logistics',
                 'domain_filter': 'logistics'
             # },
             # {
             #         'title': 'Optimal AGRD Complexity',
             #         'file_name': 'all',
             #         'domain_filter': None
             }
         ])
