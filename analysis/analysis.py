import os.path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import glob

'''
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
'''


def plot_mqa_over_time(conditions, base='../data/'):
    for condition in conditions:
        if condition != 'ET-GOA':
            continue
        paths = glob.glob(os.path.join(base, '*_primary_{}.csv'.format(condition)))
        for p in paths:
            df = pd.read_csv(p)
            d_mqa = df['mqa'].to_numpy()
            d_t = df['timestamp'].to_numpy()
            modes = df['state'].tolist()
            planning = []
            executing = []
            assessing = []
            anomaly = []
            for t, mode in zip(d_t, modes):
                if 'Executing' in mode and 'Assessing' not in mode:
                    executing.append(t)
                elif 'Assessing' in mode:
                    assessing.append(t)
                elif 'Planning' in mode:
                    planning.append(t)
                elif 'Anomaly' in mode:
                    anomaly.append(t)
            d_t = d_t[1:]
            d_mqa = d_mqa[1:]

            data_mqa = np.zeros((len(d_mqa), 4))
            data_t = np.zeros(len(d_mqa))

            i = 0
            for mqa, t in zip(d_mqa, d_t):
                mqas = mqa.split('|')
                if len(mqas) == 3:
                    data_mqa[i] = np.array([0.0, 0.0, 0.0, 0.0])
                else:
                    data_mqa[i] = np.array([float(x) for x in mqas])
                data_t[i] = t
                i += 1

            t = data_t
            plt.plot(t, data_mqa[:, 0], label='x pos')
            plt.plot(t, data_mqa[:, 1], label='y pos')
            plt.plot(t, data_mqa[:, 2], label='speed')
            plt.plot(t, data_mqa[:, 3], label='battery')

            plt.scatter(planning, [1.05] * len(planning), color='green', marker='s', label='Planning')
            plt.scatter(assessing, [1.05] * len(assessing), color='orange', marker='s', label='Assessing')
            plt.scatter(executing, [1.05] * len(executing), color='blue', marker='s', label='Executing')
            plt.scatter(anomaly, [1.05] * len(anomaly), color='red', marker='s', label='Anomaly')

            plt.ylim([0, 1.1])
            plt.legend()
            plt.title('MQA over time for {}'.format(condition))
        plt.show()


def plot_goa_over_time(conditions, base='../data/'):
    for condition in conditions:
        if condition == 'TELEM':
            continue
        paths = glob.glob(os.path.join(base, '*_primary_{}.csv'.format(condition)))
        for p in paths:
            df = pd.read_csv(p)
            d_goa = df['goa'].to_numpy()
            d_t = df['timestamp'].to_numpy()

            data_goa = np.zeros((len(d_goa), 4))
            data_t = np.zeros(len(d_goa))

            i = 0
            for goa, t in zip(d_goa, d_t):
                if type(goa) is not str:
                    data_goa[i] = np.array([0, 0, 0, 0])
                    data_t[i] = t
                else:
                    goas = goa.split('|')
                    if len(goas) == 5:
                        data_goa[i] = np.array([0.0, 0.0, 0.0, 0.0]) + [0.01, 0.02, 0.03, 0.04]
                        data_t[i] = t
                    else:
                        data_goa[i] = np.array([float(x) for x in goas]) + [0.01, 0.02, 0.03, 0.04]
                        data_t[i] = t
                i += 1

            t = data_t
            plt.plot(t, data_goa[:, 0], label='arrive POI', color='orange')
            plt.plot(t, data_goa[:, 1], label='Mission time', color='red')
            plt.plot(t, data_goa[:, 2], label='Battery', color='blue')
            plt.plot(t, data_goa[:, 3], label='Obstacles avoided', color='green')
            plt.ylim([0, 1.1])
            plt.legend()
            plt.title('GOA over time for {}'.format(condition))
            plt.show()


def plot_trust_survey_responses(conditions, base='../data/'):
    data = {c: [] for c in conditions}
    for c in conditions:
        before_paths = glob.glob(os.path.join(base, '*_trust_survey_BEFORE_{}.csv'.format(c)))
        after_paths = glob.glob(os.path.join(base, '*_trust_survey_AFTER_{}.csv'.format(c)))
        before_scores = []
        for p in before_paths:
            df = pd.read_csv(p)
            results = df['score'].to_numpy()
            before_scores.append(results[0])

        after_scores = []
        for p in after_paths:
            df = pd.read_csv(p)
            results = df['score'].to_numpy()
            after_scores.append(results[0])

        data[c] = [before_scores, after_scores]

    plot_data = []
    labels = []
    for k, v in data.items():
        plot_data.append(v[0])
        plot_data.append(v[1])
        labels.append('before\n{}'.format(k))
        labels.append('after\n{}'.format(k))

    plt.boxplot(plot_data, labels=labels)
    plt.ylim([0, 100])
    plt.title('Trust\n(TPR-HRI 14 pt subscale)')
    plt.show()
    return data


def plot_usability_scores(conditions, base='../data/'):
    data = {}
    for c in conditions:
        paths = glob.glob(os.path.join(base, '*_usability_survey_{}.csv'.format(c)))
        scores = []
        for p in paths:
            df = pd.read_csv(p)
            results = df['score'].to_numpy()
            scores.append(results[0])
        data[c] = scores

    plot_data = []
    plot_labels = []
    for k, v in data.items():
        plot_data.append(v)
        plot_labels.append(k)

    plt.boxplot(plot_data, labels=plot_labels)
    plt.ylim([0, 100])
    plt.title('Usability\n(SUS 10 pt scale)')
    plt.show()
    return data


def plot_secondary_performance(conditions, base='../data/'):
    accuracy = {}
    times = {}
    for condition in conditions:
        paths = glob.glob(os.path.join(base, '*_secondary_{}.csv'.format(condition)))
        a_data = []
        t_data = []
        for p in paths:
            df = pd.read_csv(p)
            ts = df['decision time'].to_numpy()
            act_x = df['actual_x'].to_numpy()
            act_y = df['actual_y'].to_numpy()
            act_std = df['actual_std'].to_numpy()
            rep_x = df['x'].to_numpy()
            rep_y = df['y'].to_numpy()

            act_x = act_x[ts > 0]
            act_y = act_y[ts > 0]
            act_std = act_std[ts > 0]
            rep_x = rep_x[ts > 0]
            rep_y = rep_y[ts > 0]
            ts = ts[ts > 0]

            act = np.column_stack((act_x, act_y))
            rep = np.column_stack((rep_x, rep_y))
            distances = np.linalg.norm(act - rep, axis=1)

            [a_data.append(c) for c in distances]
            [t_data.append(c) for c in ts]
        accuracy[condition] = a_data
        times[condition] = t_data

    plot_data = []
    plot_labels = []
    for k, v in accuracy.items():
        plot_data.append(v)
        plot_labels.append(k)
    plt.boxplot(plot_data, labels=plot_labels)
    plt.ylim(ymin=0)
    plt.title('Secondary task performance - accuracy distance'+'\n'+'$|LL_{act}-LL_{found}|$')
    plt.show()

    plot_data = []
    plot_labels = []
    for k, v in times.items():
        plot_data.append(v)
        plot_labels.append(k)
    plt.boxplot(plot_data, labels=plot_labels)
    plt.ylim(ymin=0)
    plt.title('Secondary task performance - reaction time'+'\n'+r'$|t_{start}-t_{found}|$')
    plt.show()
    return accuracy, times


def plot_anomaly_response_time(conditions, base='../data/'):
    data = {}
    for condition in conditions:
        paths = glob.glob(os.path.join(base, '*_primary_{}.csv'.format(condition)))
        tmp_data = []
        for p in paths:
            df = pd.read_csv(p)
            d_anomaly = df['anomaly state']
            d_anomaly.fillna('', inplace=True)
            d_anomaly = d_anomaly.tolist()
            d_t = df['timestamp'].to_numpy()
            d_help = df['mission control text']
            d_help.fillna('', inplace=True)
            d_help = d_help.tolist()

            start = -1
            end = -1
            for anomaly, req, t in zip(d_anomaly, d_help, d_t):
                if start < 0 and anomaly != '' and 'Looks like that fixed the anomaly' not in req:
                    start = t
                elif start > 0 and 'Looks like that fixed the anomaly' in req:
                    end = t
                if start > 0 and end > 0:
                    tmp_data.append(end - start)
                    start = -1
                    end = -1
        data[condition] = tmp_data
    plot_data = []
    plot_labels = []
    for k, v in data.items():
        plot_data.append(v)
        plot_labels.append(k)

    plt.boxplot(plot_data, labels=plot_labels)
    plt.ylim(ymin=0)
    plt.title('Anomaly Response Times'+'\n'+'$|t_{start}-t_{addressed}|$')
    plt.show()
    return data


def plot_mission_objectives(conditions, base='../data/'):
    data = {}
    for condition in conditions:
        paths = glob.glob(os.path.join(base, '*_primary_{}.csv'.format(condition)))
        tmp_data = []
        for p in paths:
            df = pd.read_csv(p)
            outcomes = df['mission outcomes']
            outcomes.fillna('', inplace=True)
            outcomes = outcomes.tolist()
            for o in outcomes:
                if o != '':
                    o = [int(x) for x in o.split('|')]
                    tmp_data.append(np.sum(o))
        data[condition] = tmp_data
    plot_data = []
    plot_labels = []
    for k, v in data.items():
        plot_data.append(v)
        plot_labels.append(k)

    plt.title('Mission Objective Performance - outcomes achieved')
    plt.bar(plot_labels, [np.mean(x) for x in plot_data], edgecolor='black', color='white')
    plt.errorbar([0, 1, 2], [np.mean(x) for x in plot_data], capsize=10, yerr=[np.std(x) for x in plot_data], fmt='none', color='black', elinewidth=1)
    plt.ylim(ymin=0)
    plt.show()
    return data

def plot_demographics(base):
    paths = glob.glob(os.path.join(base, '*_demographics_survey.csv'))
    ages = []
    robotics_exp = []
    gaming_exp = []
    male = []
    female = []
    other = []
    no_answer = []
    for p in paths:
        df = pd.read_csv(p)
        ages.append(df['age'].tolist()[0])
        robotics_exp.append(df['robotics experience'].tolist()[0])
        gaming_exp.append(df['gaming experience'].tolist()[0])
        gender = df['gender'].tolist()
        if 'Male' in gender:
            male.append(1)
        elif 'Female' in gender:
            female.append(1)
        elif 'Other' in gender:
            other.append(1)
        elif 'Prefer not to answer' in gender:
            no_answer.append(1)

    print('male: {}'.format(len(male)))
    print('female: {}'.format(len(female)))
    print('other: {}'.format(len(other)))
    print('Prefer not to answer: {}'.format(len(no_answer)))
    plt.boxplot([ages, robotics_exp, gaming_exp], labels=['Age', 'Robotics exp', 'Gaming exp'])
    plt.title('Demographics')
    plt.show()



def single_csv(header, data_array):
    # TODO convert each data created in main to a single CSV file
    # userid, condition, mean primary response time, mean primary objectives complete, mean secondary accuracy, mean secondary response time, before trust, after trust, usability
    pass


if __name__ == '__main__':
    base = '../data/mixed/'
    experimental_conditions = ['TELEM', 'GOA', 'ET-GOA']
    # Primary navigation and exploration task
    d1 = plot_anomaly_response_time(experimental_conditions, base=base)
    d2 = plot_mission_objectives(experimental_conditions, base=base)

    # Secondary task
    d3, d4 = plot_secondary_performance(experimental_conditions, base=base)

    # Surveys
    d5 = plot_trust_survey_responses(experimental_conditions, base=base)
    d6 = plot_usability_scores(experimental_conditions, base=base)

    # Demographics
    plot_demographics(base=base)

    #c = 'GOA'
    #userid = 'p3'
    #line = [userid, c, np.mean(d1[c]), np.mean(d2[c]), np.mean(d3[c]), np.mean(d4[c]), d5[c][0][0], d5[c][1][0], d6[c][0]]
    #print(line)
    # TODO other plots
    # plot_goa_over_time(experimental_conditions)
    # plot_mqa_over_time(experimental_conditions)
