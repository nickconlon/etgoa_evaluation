import matplotlib.pyplot as plt
import numpy as np


def make_plots(save_path=None):
    """
    Usability:
        H1.1 -> C2/C3 > C1
        H1.2 -> C3 > C2
    Performance:
        H2.1 -> C3/C2 < C1 (response time)
        H2.2 -> C3/C2 > C1 (response accuracy)
        H2.3 -> C3 > C1/C2 (concurrent task accuracy)
    Trust:
        H3.1 -> C2/C3 > C1 after planning
        H3.2 -> C2/C3 > C1 after execution
    """
    usability = [np.random.normal(loc=25, scale=10, size=10),
                 np.random.normal(loc=50, scale=10, size=10),
                 np.random.normal(loc=75, scale=10, size=10)]

    perf_response_time = [np.random.normal(loc=25, scale=5, size=10),
                          np.random.normal(loc=25, scale=5, size=10),
                          np.random.normal(loc=5, scale=5, size=10)]
    perf_response_accuracy = [np.random.normal(loc=25, scale=10, size=10),
                              np.random.normal(loc=25, scale=10, size=10),
                              np.random.normal(loc=75, scale=10, size=10)]
    perf_task_accuracy = [np.random.normal(loc=25, scale=10, size=10),
                          np.random.normal(loc=25, scale=10, size=10),
                          np.random.normal(loc=75, scale=10, size=10)]

    trust_baseline = [np.random.normal(loc=0.2, scale=0.1, size=10),
                      np.random.normal(loc=0.2, scale=0.1, size=10),
                      np.random.normal(loc=0.2, scale=0.1, size=10)]

    trust_planning = [np.random.normal(loc=0.2, scale=0.1, size=10),
                      np.random.normal(loc=0.5, scale=0.1, size=10),
                      np.random.normal(loc=0.5, scale=0.1, size=10)]

    trust_execution = [np.random.normal(loc=0.2, scale=0.1, size=10),
                       np.random.normal(loc=0.3, scale=0.1, size=10),
                       np.random.normal(loc=0.8, scale=0.1, size=10)]
    plot_trust(trust_baseline, trust_planning, trust_execution, save_path)
    plot_usability(usability, save_path)
    plot_performance(perf_response_time, perf_response_accuracy, perf_task_accuracy, save_path)


def define_box_properties(plot_name, color_code, label):
    # each plot returns a dictionary, use plt.setp()
    # function to assign the color code
    # for all properties of the box plot of particular group
    # use the below function to set color for particular group,
    # by iterating over all properties of the box plot
    for k, v in plot_name.items():
        plt.setp(plot_name.get(k), color='black')
    for patch in plot_name['boxes']:
        patch.set_facecolor(color_code)
    # use plot function to draw a small line to name the legend.
    plt.plot([], c=color_code, label=label)


def plot_performance(perf_response_time, perf_response_accuracy, perf_task_score, save_path):
    fig = plt.Figure(figsize=(8, 5))
    # create 2 - sample a 3-Dim array, that measures
    # the summer and winter rain fall amount
    # the list named ticks, summarizes or groups
    # the summer and winter rainfall as low, mid
    # and high
    ticks = ['C1', 'C2', 'C3']
    colors = ['red', 'orange', 'blue']
    # create a boxplot for two arrays separately,
    # the position specifies the location of the
    # particular box in the graph,
    # this can be changed as per your wish. Use width
    # to specify the width of the plot
    response_time = plt.boxplot(perf_response_time,
                                positions=np.array(np.arange(len(perf_response_time))) * 3.0 - 2 * 0.3,
                                widths=0.5,
                                patch_artist=True,
                                flierprops=dict(markerfacecolor=colors[0]))
    response_score = plt.boxplot(perf_response_accuracy,
                                 positions=np.array(np.arange(len(perf_response_accuracy))) * 3.0 + 0,
                                 widths=0.5,
                                 patch_artist=True,
                                 flierprops=dict(markerfacecolor=colors[1]))
    task_score = plt.boxplot(perf_task_score,
                             positions=np.array(np.arange(len(perf_task_score))) * 3.0 + 2 * 0.3,
                             widths=0.5,
                             patch_artist=True,
                             flierprops=dict(markerfacecolor=colors[2]))

    # setting colors for each groups
    define_box_properties(response_time, colors[0], 'Response time')
    define_box_properties(response_score, colors[1], 'Response score')
    define_box_properties(task_score, colors[2], 'Concurrent task score')

    # set the x label values
    plt.xticks([0, 3, 6], ticks)

    # set the limit for x axis
    plt.xlim(-2, 8)

    # set the limit for y axis
    plt.ylim(0, 100)

    plt.legend()
    plt.title('Performance')
    if save_path is not None:
        plt.savefig(save_path)
    plt.show()


def plot_usability(usability, save_path):
    fig = plt.Figure(figsize=(8, 5))
    ticks = ['C1', 'C2', 'C3']
    colors = ['red']
    usability_c1_plot = plt.boxplot(usability[0],
                                    positions=[1],
                                    widths=0.5,
                                    patch_artist=True,
                                    flierprops=dict(markerfacecolor=colors[0]))

    usability_c2_plot = plt.boxplot(usability[1],
                                    positions=[2],
                                    widths=0.5,
                                    patch_artist=True,
                                    flierprops=dict(markerfacecolor=colors[0]))

    usability_c3_plot = plt.boxplot(usability[2],
                                    positions=[3],
                                    widths=0.5,
                                    patch_artist=True,
                                    flierprops=dict(markerfacecolor=colors[0]))

    define_box_properties(usability_c1_plot, colors[0], 'Baseline')
    define_box_properties(usability_c2_plot, colors[0], 'Baseline')
    define_box_properties(usability_c3_plot, colors[0], 'Baseline')
    plt.xticks([1, 2, 3], ticks)
    plt.ylim(0, 100)
    plt.xlim(0, 4)
    plt.axhline(y=68, linestyle='--', color='black')
    plt.text(x=0.1, y=69, s='Highly usable')
    #plt.legend()
    plt.title('Subjective Usability')
    if save_path is not None:
        plt.savefig(save_path)
    plt.show()


def plot_trust(trust_baseline, trust_planning, trust_execution, save_path):
    fig = plt.Figure(figsize=(8, 5))
    # create 2 - sample a 3-Dim array, that measures
    # the summer and winter rain fall amount
    # the list named ticks, summarizes or groups
    # the summer and winter rainfall as low, mid
    # and high
    ticks = ['C1', 'C2', 'C3']
    colors = ['red', 'orange', 'blue']
    # create a boxplot for two arrays separately,
    # the position specifies the location of the
    # particular box in the graph,
    # this can be changed as per your wish. Use width
    # to specify the width of the plot
    trust_baseline_plot = plt.boxplot(trust_baseline,
                                      positions=np.array(np.arange(len(trust_baseline))) * 3.0 - 2 * 0.3,
                                      widths=0.5,
                                      patch_artist=True,
                                      flierprops=dict(markerfacecolor=colors[0]))
    trust_planning_plot = plt.boxplot(trust_planning,
                                      positions=np.array(np.arange(len(trust_planning))) * 3.0 + 0,
                                      widths=0.5,
                                      patch_artist=True,
                                      flierprops=dict(markerfacecolor=colors[1]))
    trust_execution_plot = plt.boxplot(trust_execution,
                                       positions=np.array(np.arange(len(trust_execution))) * 3.0 + 2 * 0.3,
                                       widths=0.5,
                                       patch_artist=True,
                                       flierprops=dict(markerfacecolor=colors[2]))

    # setting colors for each groups
    define_box_properties(trust_baseline_plot, colors[0], 'Baseline')
    define_box_properties(trust_planning_plot, colors[1], 'Post-planning')
    define_box_properties(trust_execution_plot, colors[2], 'Post-execution')

    # set the x label values
    plt.xticks([0, 3, 6], ticks)

    # set the limit for x axis
    plt.xlim(-2, 8)

    # set the limit for y axis
    plt.ylim(0, 1)

    plt.legend()
    plt.title('Subjective Trust')
    if save_path is not None:
        plt.savefig(save_path)
    plt.show()


if __name__ == '__main__':
    path = ""
    make_plots()
