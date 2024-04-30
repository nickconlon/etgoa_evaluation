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


def plot_trust_survey_responses(conditions, base='../data/', show=True):
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
            if len(results) > 0 and results[0] > 0:
                after_scores.append(results[0])
            else:
                print('bad result ', p)

        data[c] = [before_scores, after_scores]

    plot_data = []
    labels = []
    for k, v in data.items():
        plot_data.append(v[0])
        plot_data.append(v[1])
        labels.append('before\n{}'.format(k))
        labels.append('after\n{}'.format(k))

    if show:
        plt.boxplot(plot_data, labels=labels)
        plt.ylim([0, 100])
        plt.title('Trust\n(TPR-HRI 14 pt subscale)')
        plt.show()
    return data


def plot_usability_scores(conditions, base='../data/', show=True):
    data = {}
    for c in conditions:
        paths = glob.glob(os.path.join(base, '*_usability_survey_{}.csv'.format(c)))
        scores = []
        for p in paths:
            df = pd.read_csv(p)
            results = df['score'].to_numpy()
            if len(results) > 0:
                scores.append(results[0])
            else:
                print('bad result ', p)
        data[c] = scores

    plot_data = []
    plot_labels = []
    for k, v in data.items():
        plot_data.append(v)
        plot_labels.append(k)
    if show:
        plt.boxplot(plot_data, labels=plot_labels)
        plt.ylim([0, 100])
        plt.title('Usability\n(SUS 10 pt scale)')
        plt.show()
    return data


def plot_secondary_performance(conditions, base='../data/', show=True):
    accuracy = {}
    times = {}
    correct = {}
    skips = {}
    for condition in conditions:
        paths = glob.glob(os.path.join(base, '*_secondary_{}.csv'.format(condition)))
        a_data = []
        t_data = []
        c_data = []
        skip_data = []
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

            misses = len(ts[ts <= 0])


            ts = ts[ts > 0]

            act = np.column_stack((act_x, act_y))
            rep = np.column_stack((rep_x, rep_y))
            distances = np.linalg.norm(act - rep, axis=1)
            corr = (distances < 2*act_std)

            print('Misses {} : {}'.format(condition, (len(corr[corr > 0]))/(len(corr)+misses)))

            [a_data.append(c) for c in distances]
            [t_data.append(c) for c in ts]
            c_data.append(len(corr[corr > 0])/(len(corr)+misses))
            skip_data.append(misses/(len(corr)+misses))
            # TODO percentage correct correct/wrong
        accuracy[condition] = a_data
        times[condition] = t_data
        correct[condition] = c_data
        skips[condition] = skip_data

    plot_data = []
    plot_labels = []
    for k, v in accuracy.items():
        plot_data.append(v)
        plot_labels.append(k)
    if show:
        plt.boxplot(plot_data, labels=plot_labels, showfliers=False)
        plt.ylim(ymin=0)
        plt.title('Secondary task performance - accuracy distance'+'\n'+'$|LL_{act}-LL_{found}|$')
        plt.ylabel('Pixels')
        plt.show()

    plot_data = []
    plot_labels = []
    for k, v in times.items():
        plot_data.append(v)
        plot_labels.append(k)
    if show:
        plt.boxplot(plot_data, labels=plot_labels)#, showfliers=False)
        plt.ylim(ymin=0)
        plt.title('Secondary task performance - reaction time'+'\n'+r'$|t_{start}-t_{found}|$')
        plt.ylabel('Seconds')
        plt.show()

    plot_data = []
    plot_labels = []
    for k, v in correct.items():
        plot_data.append(v)
        plot_labels.append(k)
    if show:
        plt.bar(plot_labels, [np.mean(x) * 100 for x in plot_data], edgecolor='black', color='white')
        plt.errorbar([0, 1, 2], [np.mean(x) * 100 for x in plot_data], capsize=10, yerr=[np.std(x) * 100 for x in plot_data], fmt='none', color='black', elinewidth=1)
        plt.ylim(ymin=0)
        plt.title('Secondary task performance correct')
        plt.ylabel('% Correct')
        plt.show()

    plot_data = []
    plot_labels = []
    for k, v in skips.items():
        plot_data.append(v)
        plot_labels.append(k)
    if show:
        plt.bar(plot_labels, [np.sum(x)*100 for x in plot_data], edgecolor='black', color='white')
        plt.errorbar([0, 1, 2], [np.sum(x)*100 for x in plot_data], capsize=10, yerr=[np.std(x) for x in plot_data], fmt='none', color='black', elinewidth=1)
        plt.ylim(ymin=0, ymax=100)
        plt.title('Secondary task skips')
        plt.ylabel('% skipped')
        plt.show()

    return accuracy, times, correct, skips

def et_goa_special(d_anomaly, d_help, d_t, d_state):
    tmp_data = []
    start = -1
    end = -1
    for anomaly, req, t, state in zip(d_anomaly, d_help, d_t, d_state):
        if start < 0 and anomaly != '' and 'Looks like that fixed the anomaly' not in req:
            start = t
        elif start > 0 and 'Looks like that fixed the anomaly' in req:
            end = t
        # elif start > 0 and condition == 'TELEM' and 'Looks like that fixed the anomaly' in req:
        #    end = t
        # elif start > 0 and condition == 'GOA' and 'Looks like that fixed the anomaly' in req:
        #    end = t
        # elif start > 0 and condition == 'ET-GOA' and 'Executing | Auto | Assessing' in state:
        #    end = t
        if start > 0 and end > 0:
            tmp_data.append(end - start)
            start = -1
            end = -1

def plot_anomaly_response_time(conditions, base='../data/', show=True):
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
            for anomaly, req, t in zip(d_anomaly, d_help, d_t): #if start < 0 and anomaly != '' and 'Executing | Auto | Assessing' not in req:
                if start < 0 and anomaly != '' and 'Looks like that fixed the anomaly' not in req:
                    start = t
                elif start > 0 and 'Looks like that fixed the anomaly' in req: # elif start > 0 and state == 'Executing':
                    end = t
                #elif start > 0 and condition == 'TELEM' and 'Looks like that fixed the anomaly' in req:
                #    end = t
                #elif start > 0 and condition == 'GOA' and 'Looks like that fixed the anomaly' in req:
                #    end = t
                #elif start > 0 and condition == 'ET-GOA' and 'Executing | Auto | Assessing' in state:
                #    end = t
                if start > 0 and end > 0:
                    tmp_data.append(end - start)
                    start = -1
                    end = -1
            if start > 0 and end == -1:
                end = d_t[-1]
                tmp_data.append(end - start)
               # prev = [anomaly, req, t, state]
        data[condition] = tmp_data
    plot_data = []
    plot_labels = []
    for k, v in data.items():
        plot_data.append(v)
        plot_labels.append(k)

    et_goa_assessment_time = [0.4995890000000003, 0.49931500000000284, 0.4990399999999937, 0.49978400000000534, 0.500472000000002, 0.4994339999999937, 0.4992459999999994, 0.4998649999999998, 0.4988879999999938, 0.5003630000000072, 0.5002340000000061, 0.5003919999999908, 0.4999260000000021, 0.5008569999999963, 0.5001780000000053, 0.5004169999999988, 0.5005299999999977, 0.5004670000000004, 0.5004390000000001, 0.5000410000000102, 11.500242999999998, 0.500237999999996, 0.49953399999999704, 0.4997100000000074, 0.4996890000000036, 0.4998240000000038, 0.5006850000000043, 0.5002070000000032, 0.4997330000000062, 0.4996839999999878, 0.5000229999999988, 0.5002179999999896, 0.4999509999999958, 0.5005819999999943, 0.5001709999999946, 0.49991099999999733, 0.5003560000000107, 0.5002020000000016, 0.500702000000004, 0.500717999999992, 0.5004010000000108, 0.4990329999999972, 0.4994209999999981, 0.5005779999999902, 0.49908700000000294, 0.4998440000000244, 0.5005040000000065, 0.4995980000000202, 0.49993700000001695, 0.5000050000000158, 21.500193000000024, 0.4991899999999987, 0.5003400000000013, 0.5000389999999939, 0.5000290000000049, 0.4999600000000015, 0.49985400000000624, 0.5005720000000053, 0.500304000000007, 0.49977000000000515, 0.49983500000000447, 0.49886399999999753, 0.5005329999999972, 0.5001120000000014, 0.5003780000000049, 0.4998509999999996, 0.5008399999999966, 0.5008119999999963, 0.49957699999998795, 0.4415730000000053, 0.49975599999999076, 0.5005610000000047, 0.49939700000000187, 0.4994049999999959, 0.5000959999999992, 0.5000169999999997, 0.5363929999999897, 0.5001540000000091, 0.49981800000000476, 0.4995620000000116, 0.5000369999999918, 0.5004749999999945, 0.499652999999995, 0.5009749999999968, 0.5001340000000027, 0.4992060000000009, 0.5002240000000029, 0.4995890000000003, 0.4997630000000015, 0.4998369999999994, 0.5001079999999973, 0.5001180000000005, 0.5000730000000004, 0.4997770000000088, 9.000199999999992, 0.5002679999999913, 0.4998289999999912, 0.49982200000000887, 0.499771999999993, 0.49984400000001017, 0.500204999999994, 0.5003329999999977, 0.4994209999999981, 0.5003420000000034, 0.5012610000000137, 0.499061999999995, 0.49984100000000353, 0.49932400000000143, 0.49949299999998686, 0.5005210000000204, 0.5001440000000059, 0.5007459999999924, 0.5003870000000177, 0.5001769999999794, 0.5001220000000046, 0.49924500000000194, 5.999873000000008, 0.500220999999982, 0.49999400000000094, 0.49993700000000274, 0.49970899999999574, 0.4994580000000042, 0.5004179999999963, 0.4997939999999943, 0.5002130000000022, 0.5001069999999999, 0.5005629999999996, 0.5005340000000018, 0.4991150000000033, 0.49959400000000187, 0.49983500000000447, 0.5000450000000001, 0.4992019999999968, 11.500210999999993, 0.4997099999999932, 0.5002539999999982, 0.5004079999999931, 0.5001559999999898, 0.5001530000000116, 0.5001559999999898, 0.49913899999999956, 0.4999030000000033, 0.49984500000000764, 0.4992069999999984, 0.5006510000000048, 0.4995429999999885, 0.49962799999997287, 0.49879699999999616, 0.5001969999999858, 0.49987899999999286, 0.5005649999999946, 0.5000530000000083, 0.5003209999999854, 0.5004309999999919, 0.5002409999999884, 0.49994099999997843, 0.5003650000000164, 0.5002350000000035, 0.4997690000000148, 0.49983899999998016, 0.5006740000000036, 0.4998769999999979, 14.000928999999985, 0.5001519999999857, 0.4993510000000043, 0.4993419999999844, 0.5000530000000083, 0.4999380000000144, 0.4999509999999816, 0.5002650000000131, 0.5003509999999949, 0.4998960000000068, 0.5007190000000037, 0.5004529999999932, 0.49966499999999314, 0.5000060000000133, 0.5003200000000163, 0.5003320000000144, 0.5008909999999958, 0.4994000000000085, 0.4998239999999896, 0.544148000000007, 0.5000980000000084, 0.4995169999999973, 0.5009190000000103, 0.5002399999999909, 0.4995020000000068, 0.5009970000000123, 0.500567999999987, 0.5003089999999872, 0.5007150000000138, 0.500425000000007, 0.5000339999999994, 0.5001719999999921, 0.5000849999999843, 0.4999690000000214, 0.500427000000002, 0.4999430000000018, 19.00050999999999, 0.5004759999999919, 0.5006349999999884, 0.49971299999998564, 0.49946800000000735, 0.49989199999998846, 0.4998600000000124, 0.49924900000002026, 0.49996299999997973, 0.5001370000000236, 0.5003080000000182, 0.500425000000007, 0.4994399999999928, 0.5001460000000009, 0.4999480000000176, 0.4998779999999954, 0.5001959999999883, 0.49982700000001046, 0.4999270000000138, 0.4997099999999932, 0.4991550000000018, 0.4996940000000052, 0.5008219999999994, 11.000324000000006, 0.500300999999979, 0.49936699999997813, 0.5008759999999768, 0.4995169999999973, 0.5006640000000004, 0.49926700000000324, 0.5006290000000035, 0.49973200000002294, 0.5001549999999781, 0.49987500000000296, 0.5002919999999733, 0.500912999999997, 13.500001999999995, 0.5005430000000075, 0.49931800000000237, 0.4996670000000023, 0.4999090000000024, 0.5002120000000048, 0.5000520000000108, 0.500550000000004, 0.5003560000000107, 0.49959400000000187, 0.4999699999999905, 0.4994920000000036, 0.5002939999999967, 0.499369999999999, 0.49961199999998485, 0.5000079999999798, 0.49914199999997777, 0.501119999999986, 0.5001120000000014, 0.5002080000000149, 0.4999260000000163, 0.5001440000000059, 22.499998000000005, 0.5000239999999962, 0.5000670000000014, 0.49876199999999926, 0.49926199999998744, 0.4998939999999834, 0.4998119999999915, 0.5005420000000242, 0.4998160000000098, 0.500128999999987, 0.49941599999999653, 0.4994429999999852, 0.5007769999999994, 0.4991149999999891, 0.5002350000000035, 0.4994520000000193, 0.5002750000000162, 0.4998000000000218, 0.4994220000000098, 0.4999270000000138, 0.5002410000000168, 0.49905999999998585, 0.4999699999999905, 0.5004729999999995, 0.49943799999999783, 0.49987600000000043, 0.499732999999992, 0.5004619999999989, 8.999266000000006, 0.5008899999999983, 0.5006029999999839, 0.5003510000000233, 0.4999470000000201, 0.49921699999998737, 0.49970400000000836, 0.49960900000002084, 0.500079999999997, 0.49993200000000115, 0.4999610000000132, 0.5005039999999781, 0.5006480000000124, 0.50030000000001, 0.49982800000000793, 0.5005569999999864, 0.49995400000000245, 0.5006900000000201, 8.50057799999999, 0.5004069999999956]
    et_got_automation_response_time = [0.4995890000000003, 0.49931500000000284, 0.4990399999999937, 0.49978400000000534, 0.500472000000002, 0.4994339999999937, 0.4992459999999994, 0.4998649999999998, 0.4988879999999938, 36.00058100000001, 115.500441, 10.500624000000002, 0.5001120000000014, 0.5003780000000049, 0.4998509999999996, 0.5008399999999966, 0.5008119999999963, 0.49957699999998795, 48.941325000000006, 0.49999400000000094, 0.49993700000000274, 0.49970899999999574, 0.4994580000000042, 0.5004179999999963, 0.4997939999999943, 0.5002130000000022, 47.999894, 120.500595, 10.999303999999995, 0.4995179999999948, 0.49980799999997316, 0.4997060000000033, 0.49977400000000216, 0.5001179999999863, 0.5002750000000162, 45.95620099999999, 23.99998100000002, 40.00033300000004, 82.50023, 0.49876199999999926, 0.49926199999998744, 0.4998939999999834, 0.4998119999999915, 0.5005420000000242, 0.4998160000000098, 0.500128999999987, 5.000396999999992, 0.5003419999999892, 0.5000439999999742, 0.5002669999999796, 0.5002739999999903, 0.499859999999984, 0.5004950000000008, 0.4999910000000227, 0.5002240000000029, 14.499902999999989, 0.5006029999999839, 0.5003510000000233, 0.4999470000000201, 0.49921699999998737, 0.49970400000000836, 0.49960900000002084, 0.500079999999997, 0.49993200000000115, 0.4999610000000132]
    ind = np.arange(3)
    plt.bar(ind, [np.mean(x) for x in plot_data], edgecolor='black', color='white')
    plt.bar([ind[-1]], [np.mean(et_got_automation_response_time)+np.mean(et_goa_assessment_time)], edgecolor='black', color='white')
    plt.bar([ind[-1]], [np.mean(et_got_automation_response_time)], edgecolor='black', color='white')

    # TODO hard coded values
    plt.annotate('{:.0f}s'.format(np.mean(et_got_automation_response_time)), (ind[-1]+0.2, 3))
    plt.annotate('{:.0f}s'.format(np.mean(et_goa_assessment_time)), (ind[-1]+0.2, 12))
    plt.annotate('{:.0f}s'.format(np.mean(plot_data[-1])-np.mean(et_got_automation_response_time)-np.mean(et_goa_assessment_time)), (ind[-1]+0.2, 25))

    plt.annotate('{:.0f}s'.format(np.mean(plot_data[0])), (ind[0]-0.2, 42))
    plt.annotate('{:.0f}s'.format(np.mean(plot_data[1])), (ind[1]-0.2, 60))
    plt.annotate('{:.0f}s'.format(np.mean(plot_data[2])), (ind[2]-0.2, 25))


    plt.xticks(ind, plot_labels)


    #
    plt.errorbar([0, 1, 2], [np.mean(x) for x in plot_data], capsize=10, yerr=[np.std(x) for x in plot_data], fmt='none', color='black', elinewidth=1)
    #plt.boxplot(plot_data, labels=plot_labels, showfliers=False)
    #plt.boxplot([et_got_automation_response_time], labels=[''], showfliers=False, positions=[3], widths=[0.3])
    plt.ylim(ymin=0)
    plt.ylabel('Seconds')
    plt.title('Anomaly Response Times'+'\n'+'$|t_{start}-t_{addressed}|$')
    if show:
        plt.show()
    return data


def plot_mission_objectives(conditions, base='../data/', show=True):
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
                    tmp_data.append(np.sum(o) / 4 * 100)
        data[condition] = tmp_data
    plot_data = []
    plot_labels = []
    for k, v in data.items():
        plot_data.append(v)
        plot_labels.append(k)

    if show:
        plt.title('Mission Objective Performance - outcomes achieved')
        plt.bar(plot_labels, [np.mean(x) for x in plot_data], edgecolor='black', color='white')
        plt.errorbar([0, 1, 2], [np.mean(x) for x in plot_data], capsize=10, yerr=[np.std(x) for x in plot_data], fmt='none', color='black', elinewidth=1)
        plt.ylim(ymin=0)
        plt.ylabel('% Objectives achieved')
        plt.show()
    return data

def plot_demographics(base, show=True):
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
    if show:
        fig, (a1, a2) = plt.subplots(nrows=1, ncols=2)  # , figsize=(8, 8))
        a1.boxplot([ages, robotics_exp, gaming_exp], labels=['Age', 'Robotics\nexp', 'Gaming\nexp'])
        a2.bar(['Male', 'Female', 'Other', 'No\nanswer'], [len(male), len(female), len(other), len(no_answer)], edgecolor='black', color='white')
        a2.set_ylim([0, 8])
        fig.suptitle('Demographics')
        plt.tight_layout()
        plt.show()

def plot_decisions():
    paths = glob.glob(os.path.join(base, '*_decisions_survey.csv'))
    print('********** DURING PLANNING ******************')
    map = 0
    telem = 0
    camera = 0
    assmt = 0
    other = 0
    for p in paths:
        df = pd.read_csv(p)
        responses = df['response'].to_list()
        if len(responses) > 5:
            map = map + 1 if responses[0] == 'True' else map
            telem = telem + 1 if responses[1] == 'True' else telem
            camera = camera + 1 if responses[2] == 'True' else camera
            assmt = assmt + 1 if responses[3] == 'True' else assmt
            other = other + 1 if responses[4] == 'True' else other
            if type(responses[5]) is str:
                print(responses[5])
        else:
            print('bad response ', p)

    plot_labels = ['Map', 'Telem', 'Cam', 'Assmt', 'Other']
    plot_data = [map, telem, camera, assmt, other]
    plt.bar(plot_labels, plot_data, edgecolor='black', color='white')
    plt.title('Widget usage during planning')
    plt.show()

    print('********** DURING EXECUTION ******************')
    map = 0
    telem = 0
    camera = 0
    assmt = 0
    other = 0
    for p in paths:
        df = pd.read_csv(p)
        responses = df['response'].to_list()
        if len(responses) > 5:
            map = map + 1 if responses[6] == 'True' else map
            telem = telem + 1 if responses[7] == 'True' else telem
            camera = camera + 1 if responses[8] == 'True' else camera
            assmt = assmt + 1 if responses[9] == 'True' else assmt
            other = other + 1 if responses[10] == 'True' else other
            if type(responses[5]) is str:
                print(responses[5])
        else:
            print('bad response ', p)

    plot_labels = ['Map', 'Telem', 'Cam', 'Assmt', 'Other']
    plot_data = [map, telem, camera, assmt, other]
    plt.bar(plot_labels, plot_data, edgecolor='black', color='white')
    plt.title('Widget usage during execution')
    plt.show()





def single_csv():
    header = ['userid', 'condition', 'mean primary response time', 'mean primary objectives complete',
              'mean secondary accuracy', 'mean secondary response time', 'mean secondary correctness',
              'secondary skips',
              'before trust', 'after trust', 'usability']
    lines = []
    participants = ['p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9', 'p10']
    conditions = ['TELEM', 'GOA', 'GOA', 'ET-GOA', 'ET-GOA', 'TELEM', 'TELEM', 'GOA', 'TELEM', 'GOA']
    for participant, condition in zip(participants, conditions):
        base = '../data/{}/'.format(participant)
        experimental_conditions = ['TELEM', 'GOA', 'ET-GOA']
        # Primary navigation and exploration task
        d1 = plot_anomaly_response_time(experimental_conditions, base=base, show=False)
        d2 = plot_mission_objectives(experimental_conditions, base=base, show=False)

        # Secondary task
        acc, tim, corr, skips = plot_secondary_performance(experimental_conditions, base=base, show=False)

        # Surveys
        d5 = plot_trust_survey_responses(experimental_conditions, base=base, show=False)
        d6 = plot_usability_scores(experimental_conditions, base=base, show=False)

        # Demographics
        plot_demographics(base=base, show=False)
        c = condition
        if participant == 'p2':
            line = [participant, c, np.mean(d1[c]), np.mean(d2[c]), np.mean(acc[c]), np.mean(tim[c]),
                    np.mean(corr[c]), np.sum(skips[c]), d5[c][1][0], d5[c][1][0], d6[c][0]]
        else:
            line = [participant, c, np.mean(d1[c]), np.mean(d2[c]), np.mean(acc[c]), np.mean(tim[c]),
                    np.mean(corr[c]), np.sum(skips[c]), d5[c][0][0], d5[c][1][0], d6[c][0]]
        lines.append(line)
    df = pd.DataFrame(data=lines, columns=header)
    print(df)
    df.to_csv('test.csv', index=False)

    pass


if __name__ == '__main__':
    #single_csv()
    #print(0/0)
    base = '../data/'
    experimental_conditions = ['TELEM', 'GOA', 'ET-GOA']
    # Primary navigation and exploration task
    d1 = plot_anomaly_response_time(experimental_conditions, base=base)
    d2 = plot_mission_objectives(experimental_conditions, base=base)

    # Secondary task
    d3, d4, correct, skips = plot_secondary_performance(experimental_conditions, base=base)

    # Surveys
    d5 = plot_trust_survey_responses(experimental_conditions, base=base)
    d6 = plot_usability_scores(experimental_conditions, base=base)

    # Demographics
    plot_demographics(base=base)

    #plot_decisions()
    #test = []
    #p_telem = np.asarray(d2['TELEM']) * 1/(1+np.asarray(d1['TELEM'])) + np.asarray(corr['TELEM']) * 1/(1+np.asarray(skips['TELEM']))
    #p_goa = [0]#np.asarray(d2['GOA']) * 1/(1+np.asarray(d1['GOA'])) + np.asarray(corr['GOA']) * 1/(1+np.asarray(skips['GOA']))
    #p_etgoa = np.asarray(d2['ET-GOA']) * 1/(1+np.asarray(d1['ET-GOA'])) + np.asarray(corr['ET-GOA']) * 1/(1+np.asarray(skips['ET-GOA']))

    #p_telem = np.multiply(np.asarray(d2['TELEM']), np.asarray(corr['TELEM'])) * 1/(1+np.asarray(skips['TELEM']))
    #p_goa = np.multiply(np.asarray(d2['GOA']), np.asarray(corr['GOA'])) * 1/(1+np.asarray(skips['GOA']))
    #p_etgoa = np.multiply(np.asarray(d2['ET-GOA']), np.asarray(corr['ET-GOA'])) * 1/(1+np.asarray(skips['ET-GOA']))

    #print('\n\n')
    #print('Condition, performance')
    #for p in p_telem:
    #    print('Telem, {}'.format(p))
    #for p in p_goa:
    #    print('GOA, {}'.format(p))
    #for p in p_etgoa:
    #    print('ET-GOA, {}'.format(p))

    #plt.boxplot([p_telem, p_goa, p_etgoa], labels=['TELEM', 'GOA', 'ET-GOA'])#, showfliers=False)
    #plt.show()

    #c = 'GOA'
    #userid = 'p3'
    #line = [userid, c, np.mean(d1[c]), np.mean(d2[c]), np.mean(d3[c]), np.mean(d4[c]), d5[c][0][0], d5[c][1][0], d6[c][0]]
    #print(line)
    # TODO other plots
    # plot_goa_over_time(experimental_conditions)
    # plot_mqa_over_time(experimental_conditions)
