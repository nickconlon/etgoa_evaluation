import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import famsec.factorized_machine_self_confidence as famsec


def find_nearest(array, value):
    idx = (np.abs(array - value)).argmin()
    return idx


def min_diff_pos_sorted(sorted_array, target):
    idx = np.searchsorted(sorted_array, target)
    idx1 = max(0, idx - 1)
    return np.abs(np.array(sorted_array[idx1:idx + 1]) - target).argmin() + idx1


def gaussian_si_1d(pred_mu, pred_std, actual, min_std=1.0, plot=False):
    """
    Compute a Surprise Index (SI) for Gaussian 1D distribution.

    :param pred_mu:     Predicted state mean
    :param pred_std:    Predicted state standard deviation
    :param actual:      Actual state
    :param min_std:     Minimum state standard deviation (inflation term)
    :param plot:        Will render plots if True, wont' otherwise
    :return:            SI for each state element [SI(x), SI(y), SI(v), SI(b)]
    """
    if pred_std == 0:
        return 0
    _myclip_a = min(actual - 10, pred_mu - 10)
    _myclip_b = max(actual + 10, pred_mu + 10)
    _loc = pred_mu
    _scale = np.maximum(pred_std, min_std)
    _a, _b = (_myclip_a - _loc) / _scale, (_myclip_b - _loc) / _scale
    _model = stats.norm(loc=_loc, scale=_scale)
    _x = np.linspace(_myclip_a, _myclip_b, num=500)
    _dist = abs(_loc - actual)
    _si = _model.cdf(_loc - _dist) + (1 - _model.cdf(_loc + _dist))
    if plot:
        y = _model.pdf(_x)
        plt.plot(_x, y, color='black')
        plt.plot([min(_x), max(_x)], [0, 0], color='black')
        plt.scatter([_loc - _dist, _loc + _dist],
                    [_model.pdf(_loc - _dist), _model.pdf(_loc + _dist)], c='red')
        plt.plot([_loc + _dist, _loc + _dist], [0, _model.pdf(_loc + _dist)], c='red')
        plt.plot([_loc - _dist, _loc - _dist], [0, _model.pdf(_loc - _dist)], c='red')
        plt.title("full:{:.2f}, SI:{:.2f}, std:{:.2f}".format(_model.cdf(_myclip_b), _si, _scale))
        plt.xlim([min(_x), max(_x)])
        plt.tight_layout()
        plt.pause(0.1)
        plt.clf()
    return _si


def kde_assessment(actual, predicted, ax=None):
    """
    Compute a Surprise Index (SI) using KDE for non-parametric distributions.

    :param actual:      Actual state
    :param predicted:   Predicted state
    :param ax:          Axis for plotting
    :return:            SI metric
    """
    dpredicted = predicted[~np.isnan(predicted)]
    if len(dpredicted) == 0:
        return 0.0
    if len(np.unique(dpredicted)) == 1:
        dpredicted = np.array([dpredicted[0] - 5, dpredicted[0], dpredicted[0] + 5])
    kernel = stats.gaussian_kde(dpredicted, bw_method=100)
    xx = np.linspace(min(dpredicted) - 5, max(dpredicted) + 5, 500)
    smaller = np.where(kernel(xx) < kernel(actual))
    p_actual = kernel(actual)[0]
    p_distribution = kernel(xx)
    p_distribution_smaller = p_distribution[smaller]

    surprise = np.trapz(p_distribution_smaller, xx[smaller])
    if ax is not None:
        ax.clear()
        ax.plot(xx, p_distribution)
        # ax.hist(dpredicted, density=True)
        ax.plot([actual, actual], [0, p_actual], color='red', linewidth=3)
        # ax.plot(xx[smaller], p_distribution_smaller)
        ax.set_title('SI={:.2f}'.format(surprise))
    return surprise


class et_goa:
    """
    Simple class to demonstrate the ET-GOA algorithm.
    """
    def __init__(self, min_stds):
        self.pred_paths = None
        self.data = None
        self.counter = 0
        self.t_start = None
        self.min_stds = min_stds
        self.texts = ['t', 'x', 'y', 'v', 'b']
        self.indices = [11, 0, 1, 7, 8]  # mission time, px, py, speed, battery

    def has_data(self):
        """
        True if World Model data is set False otherwise.
        """
        return self.data is not None

    def set_pred_paths(self, paths):
        """
        Set World Model rollout dataset paths.

        :param paths: string path to dataset
        """
        self.pred_paths = paths

    def preprocess(self):
        """Set ET-GOA's buffer as a set of World Model rollouts.

        Transform a predicted state distribution were
            s=[t, x, y, speed, battery_level]
        to:
            d=[t_mu, t_std, x_mu, x_std, y_mu, y_std, speed_mu, speed_std, batt_mu, batt_std]
        """
        print('processing rollouts')
        predicted_states = [np.load(d, allow_pickle=True) for d in self.pred_paths]
        for idx, x in enumerate(predicted_states):
            predicted_states[idx][:, 0:-2] = predicted_states[idx][:, 0:-2].astype('float64') # the last column is strings
        max_len = np.max([len(x) for x in predicted_states])
        preprocessed = np.zeros((max_len, 2*len(self.indices))) * np.nan
        for t in range(max_len):
            data = []
            for i in self.indices:
                d1 = []
                for pred in predicted_states:
                    if t >= pred.shape[0]:
                        continue
                    d1.append(pred[t, i])
                data.append([np.mean(d1), np.std(d1)])
            preprocessed[t] = np.array(data).flatten()
        #  [t_mu, t_std, x_mu, x_std, y_mu, y_std, speed_mu, speed_std, batt_mu, batt_std]
        self.counter = 0
        self.data = preprocessed

    def set_start_time(self, t_start):
        """
        Set start time

        :param t_start: Start time seconds
        """
        if self.t_start is None:
            self.t_start = t_start
            self.data[:, 0] += t_start

    def forget_start_time(self):
        """Reset start time."""
        self.t_start = None

    def get_si(self, actual_x, actual_y, actual_v, actual_b, t_now):
        """
        Compute the Surprise Index (SI) using some statistical model (here hard coded to Gaussian 1D).

        :param actual_x:    observed x position of the robot
        :param actual_y:    observed y position of the robot
        :param actual_v:    observed velocity of the robot
        :param actual_b:    observed battery level of the robot
        :param t_now:       current time
        :return:            SI for each state element [SI(x), SI(y), SI(v), SI(b)]
        """
        idx = find_nearest(self.data[:, 0], t_now)
        pred = self.data[idx]
        actuals = [actual_x, actual_y, actual_v, actual_b]
        pred_mus = [pred[2], pred[4], pred[6], pred[8]]
        pred_stds = [pred[3], pred[5], pred[7], pred[9]]
        sis = []
        for actual, pred_mu, pred_std, min_std in zip(actuals, pred_mus, pred_stds, self.min_stds):
            si = gaussian_si_1d(pred_mu, pred_std, actual, min_std)
            sis.append(si)
        print("t={:.2f} | mu(x,y,v,b): ({:.2f}, {:.2f}, {:.2f}, {:.2f}) || act({:.2f}, {:.2f}, {:.2f}, {:.2f})".
              format(t_now, pred_mus[0], pred_mus[1], pred_mus[2], pred_mus[3],
                     actuals[0], actuals[1], actuals[2], actuals[3]))
        print("t={:.2f} | st(x,y,v,b): ({:.2f}, {:.2f}, {:.2f}, {:.2f}) || min({:.2f}, {:.2f}, {:.2f}, {:.2f})".
              format(t_now, pred_stds[0], pred_stds[1], pred_stds[2], pred_stds[3],
                     self.min_stds[0], self.min_stds[1], self.min_stds[2], self.min_stds[3]))
        print('si(x): {:.2f}, si(y): {:.2f}, si(v): {:.2f}, si(b): {:.2f}'.format(*sis))
        return sis

    def get_goa_times(self, time_cutoff, time_completed):
        predicted_states = [np.load(d, allow_pickle=True) for d in self.pred_paths]
        times = [d[-1, -2] + time_completed for d in predicted_states]
        distribution = [int(x < time_cutoff) for x in times]
        partition = np.array([-2, 0, 2])
        z_star = 2
        goa_time = famsec.assess_rollouts(distribution=distribution, bins=partition, z_star=z_star)
        return goa_time


if __name__ == '__main__':
    pred_paths = ['/data/webots/rollout{}_state.npy'.format(x) for x in np.arange(0, 10)]
    etgoa = et_goa([2, 2, 0.5, 0.5])
    etgoa.set_pred_paths(pred_paths)
    etgoa.preprocess()
    etgoa.set_start_time(0)
    batt = 100
    dt = 1
    for i in range(100):
        idx = find_nearest(etgoa.data[:, 0], i)
        act = etgoa.data[idx].copy()
        act[8] = batt
        mqa = etgoa.get_si(actual_x=act[2], actual_y=act[4], actual_v=act[6], actual_b=act[8], t_now=i)
        print(mqa)
        print(act[0], i)
        batt = float(np.maximum(batt - dt/2, 0.0))

