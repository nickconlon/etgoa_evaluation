import time

import numpy as np
import yaml
from subprocess import Popen
import PyQt5.QtCore as QtCore
import traceback


def do_rollout(position, orientation, goal, waypoint_counter, known_obstacles, waypoints=None):
    """
    TODO obstacle location + radius + perturbation model (or type)

    :param position:            array [x, y]
    :param orientation:         array [x, y, z, w]
    :param goal:                array [x, y]
    :param waypoint_counter:    0
    :param known_obstacles:     {}
    :param waypoints:           array [[x0, y0],...[xn, yn]]
    :return:
    """
    d = {
        'start_position': [float(x) for x in position],
        'start_orientation': [float(x) for x in orientation],
        'goal_position': [float(x) for x in goal],
        'waypoint_index': waypoint_counter,
        'num_iterations': 5,
        'publish': False,
        'prefix': 'rollout',
        'known_obs': known_obstacles,
    }
    if waypoints is not None:
        d['wp_x'] = [float(x) for x in waypoints[:, 0]]
        d['wp_y'] = [float(x) for x in waypoints[:, 1]]

    print(yaml.dump(d))
    fname = '/home/cohrint-skynet/Documents/etgoa_eval/controllers/rollout_controller/'
    with open(fname + 'settings.yaml', 'w') as f:
        yaml.dump(d, f, default_flow_style=None, sort_keys=False)

    p = Popen('/home/cohrint-skynet/Documents/etgoa_eval/controllers/do_rollouts.sh')
    stdout, stderr = p.communicate()
    print('TODO read in monte carlo data')


class RolloutThread(QtCore.QThread):
    finished = QtCore.pyqtSignal()
    pose = None
    orientation = None
    goal = None
    known_obstacles = None
    waypoints = []

    def run(self):
        print('starting rollout thread')
        t1 = time.time()
        try:
            do_rollout(self.pose, self.orientation, self.goal, 0, {}, self.waypoints)
            self.finished.emit()
        except Exception as e:
            traceback.print_exc()
        t2 = time.time()
        print('exiting rollout thread. Secs: {:.2f}'.format(t2-t1))


def example_rollout():
    pos = [0, 0, 0]
    orientation = [0, 0, 1, 0]
    goal = [3, 3]
    known_obs = {}
    waypoints = np.array([[0, 0], [3, 3]])
    do_rollout(pos, orientation, goal, 0, known_obs, waypoints)


if __name__ == '__main__':
    example_rollout()
