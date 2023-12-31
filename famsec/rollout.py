import time

import numpy as np
import yaml
from subprocess import Popen
import PyQt5.QtCore as QtCore
import traceback
from famsec.outcomes import compute_outcomes


def do_rollout(position, orientation, goal, known_obstacles, waypoints, max_time, num_iterations):
    """
    TODO obstacle location + radius + perturbation model (or type)

    :param num_iterations:
    :param max_time:
    :param position:            array [x, y]
    :param orientation:         array [x, y, z, w]
    :param goal:                array [x, y]
    :param known_obstacles:     {}
    :param waypoints:           array [[x0, y0],...[xn, yn]]
    :return:
    """
    d = {
        'start_position': [float(x) for x in position],
        'start_orientation': [float(x) for x in orientation],
        'goal_position': [float(x) for x in goal],
        'waypoint_index': 0,
        'num_iterations': num_iterations,
        'publish': False,
        'prefix': 'rollout',
        'known_obs': known_obstacles,
        'max_time': max_time
    }
    if waypoints is not None:
        d['wp_x'] = [float(x) for x in waypoints[:, 0]]
        d['wp_y'] = [float(x) for x in waypoints[:, 1]]

    # TODO fix mess of hard coded paths
    base = '/home/cohrint-skynet/Code/etgoa_evaluation/world_model/'
    print(yaml.dump(d))
    fname = base + 'controllers/rollout_controller/'
    with open(fname + 'settings.yaml', 'w') as f:
        yaml.dump(d, f, default_flow_style=None, sort_keys=False)

    p = Popen(base + 'do_rollouts.sh')
    stdout, stderr = p.communicate()
    goas = compute_outcomes()
    return goas


class RolloutThread(QtCore.QThread):
    finished = QtCore.pyqtSignal(object)
    pose = None
    orientation = None
    goal = None
    known_obstacles = {}
    waypoints = []
    num_iterations = 10
    max_time = 200  # seconds

    def run(self):
        print('starting rollout thread')
        t1 = time.time()
        try:
            goas = do_rollout(self.pose, self.orientation, self.goal, self.known_obstacles,
                              self.waypoints, self.max_time, self.num_iterations)
            self.finished.emit(goas)
        except Exception as e:
            traceback.print_exc()
        t2 = time.time()
        print('exiting rollout thread. Secs: {:.2f}'.format(t2-t1))


def example_rollout():
    pos = [0, 0, 0]
    orientation = [0, 0, 1, 0]
    goal = [10, 10]
    known_obs = {}
    waypoints = np.array([[0, 0], goal])
    do_rollout(pos, orientation, goal, known_obs, waypoints, 200, 20)


if __name__ == '__main__':
    example_rollout()
