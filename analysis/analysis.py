import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def make_plots(df, save_path=None):
    """
    Note this is just generating fake data to get the plots right

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
    trust_baseline = [df.loc[df['condition'] == c]['baseline trust'].tolist() for c in [0, 1, 2]]
    trust_planning = [df.loc[df['condition'] == c]['post planning trust'].tolist() for c in
                      [0, 1, 2]]
    trust_execution = [df.loc[df['condition'] == c]['post execution trust'].tolist() for c in
                       [0, 1, 2]]
    plot_trust(trust_baseline, trust_planning, trust_execution, save_path)

    usability = [df.loc[df['condition'] == c]['usability'].tolist() for c in [0, 1, 2]]
    plot_usability(usability, save_path)

    anomaly_resp_time = [df.loc[df['condition'] == c]['anomaly resp'].tolist() for c in [0, 1, 2]]
    anomaly_resp_acc = [df.loc[df['condition'] == c]['anomaly accuracy'].tolist() for c in
                        [0, 1, 2]]
    task_score = [df.loc[df['condition'] == c]['task score'].tolist() for c in [0, 1, 2]]
    plot_performance(anomaly_resp_time, anomaly_resp_acc, task_score, save_path)


def define_box_properties(plot_name, color_code, label):
    for k, v in plot_name.items():
        plt.setp(plot_name.get(k), color='black')
    for patch in plot_name['boxes']:
        patch.set_facecolor(color_code)
    plt.plot([], c=color_code, label=label)


def plot_performance(perf_response_time, perf_response_accuracy, perf_task_score, save_path):
    fig = plt.Figure(figsize=(8, 5))
    ticks = ['C1', 'C2', 'C3']
    colors = ['red', 'orange', 'blue']

    response_time = plt.boxplot(perf_response_time,
                                positions=np.array(
                                    np.arange(len(perf_response_time))) * 3.0 - 2 * 0.3,
                                widths=0.5,
                                patch_artist=True,
                                flierprops=dict(markerfacecolor=colors[0]))
    response_score = plt.boxplot(perf_response_accuracy,
                                 positions=np.array(
                                     np.arange(len(perf_response_accuracy))) * 3.0 + 0,
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

    # set the limit for x-axis
    plt.xlim(-2, 8)

    # set the limit for y-axis
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
    # plt.legend()
    plt.title('Subjective Usability')
    if save_path is not None:
        plt.savefig(save_path)
    plt.show()


def plot_trust(trust_baseline, trust_planning, trust_execution, save_path):
    fig = plt.Figure(figsize=(8, 5))

    ticks = ['C1', 'C2', 'C3']
    colors = ['red', 'orange', 'blue']

    trust_baseline_plot = plt.boxplot(trust_baseline,
                                      positions=np.array(
                                          np.arange(len(trust_baseline))) * 3.0 - 2 * 0.3,
                                      widths=0.5,
                                      patch_artist=True,
                                      flierprops=dict(markerfacecolor=colors[0]))
    trust_planning_plot = plt.boxplot(trust_planning,
                                      positions=np.array(np.arange(len(trust_planning))) * 3.0 + 0,
                                      widths=0.5,
                                      patch_artist=True,
                                      flierprops=dict(markerfacecolor=colors[1]))
    trust_execution_plot = plt.boxplot(trust_execution,
                                       positions=np.array(
                                           np.arange(len(trust_execution))) * 3.0 + 2 * 0.3,
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


def plot_mqa_over_time(data):
    """
    Plot model quality assessment in [0, 1] over time
    Plot state (planning, assessing, executing) over time

    :param data:    the dataset
    """
    d_mqa = data['mqa'].to_numpy()
    d_t = data['timestamp'].to_numpy()
    modes = data['mission mode'].tolist()
    planning = []
    executing = []
    assessing = []
    for t, mode in enumerate(modes):
        if mode == 'Execution':
            executing.append(t)
        elif mode == 'Assessing':
            assessing.append(t)
        elif mode == 'Planning':
            planning.append(t)
    d_t = d_t[1:]
    d_mqa = d_mqa[1:]

    data_mqa = np.zeros((len(d_mqa), 4))
    data_t = np.zeros(len(d_mqa))

    i = 0
    for mqa, t in zip(d_mqa, d_t):
        mqas = mqa.split('_')
        if len(mqas) == 3:
            data_mqa[i] = np.array([0.0, 0.0, 0.0, 0.0])
        else:
            data_mqa[i] = np.array([float(x) for x in mqas])
        data_t[i] = t
        i += 1
    t = np.arange(0, len(data_t))

    plt.plot(t, data_mqa[:, 0], label='x pos')
    plt.plot(t, data_mqa[:, 1], label='y pos')
    plt.plot(t, data_mqa[:, 2], label='speed')
    plt.plot(t, data_mqa[:, 3], label='battery')

    plt.scatter(planning, [1.05] * len(planning), color='green', marker='s', label='Planning')
    plt.scatter(assessing, [1.05] * len(assessing), color='red', marker='s', label='Assessing')
    plt.scatter(executing, [1.05] * len(executing), color='blue', marker='s', label='Executing')

    plt.ylim([0, 1.1])
    plt.legend()
    plt.show()


def plot_goa_over_time(df):
    """

    :param df:
    :return:
    """
    d_goa = df['goa'].to_numpy()
    d_t = df['timestamp'].to_numpy()

    data_goa = np.zeros((len(d_goa), 5))
    data_t = np.zeros(len(d_goa))

    i = 0
    for goa, t in zip(d_goa, d_t):
        if type(goa) is not str:
            data_goa[i] = np.array([0, 0, 0, 0, 0])
            data_t[i] = t
        else:
            goas = goa.split('_')
            data_goa[i] = np.array([float(x) for x in goas])
            data_t[i] = t
        i += 1

    t = np.arange(0, len(data_t))
    plt.plot(t, data_goa[:, 0], label='x pos')
    plt.plot(t, data_goa[:, 1], label='y pos')
    plt.plot(t, data_goa[:, 2], label='speed')
    plt.ylim([0, 1.1])
    plt.legend()
    plt.show()


def plot_survey_responses(df):
    pass


if __name__ == '__main__':
    run_data = pd.read_csv('../data/20240116_160630_primary.csv')
    plot_goa_over_time(run_data)
    plot_mqa_over_time(run_data)
    survey_data = pd.read_csv('../data/20240116_160630_survey.csv')
    plot_survey_responses(survey_data)
