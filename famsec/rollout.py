import time

import numpy as np
import yaml
from subprocess import Popen
import PyQt5.QtCore as QtCore
import traceback
from famsec.outcomes import compute_outcomes


def do_rollout(position, orientation, goal, batt_level, batt_rate, vel_rate,
               known_obstacles, time_offset, waypoints, max_time, num_iterations,
               wm_settings_path, wp_executable_path):
    """
    :param batt_rate:           rate of battery drain per second s/t b_t+1 = b_t-rate*dt+noise
    :param batt_level:          battery starting level
    :param num_iterations:      number of rollouts
    :param max_time:            max time for a single rollout
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
        'max_time': max_time,
        'batt_level': batt_level,
        'batt_rate': batt_rate,  # units per second
        'vel_rate': vel_rate,  # percent of mean velocity
    }
    if waypoints is not None:
        d['wp_x'] = [float(x) for x in waypoints[:, 0]]
        d['wp_y'] = [float(x) for x in waypoints[:, 1]]

    # TODO fix mess of hard coded paths
    settings_fname = wm_settings_path #'/data/webots/settings.yaml'

    with open(settings_fname, 'w') as f:
        yaml.dump(d, f, default_flow_style=None, sort_keys=False)
    cmd = ['webots',
           wp_executable_path, #'/home/cohrint-skynet/catkin_ws/src/etgoa_evaluation/world_model/worlds/rollout_simulation.wbt',
           '--minimize', '--batch', '--mode=fast', '--stdout']
    p = Popen(cmd)
    stdout, stderr = p.communicate()
    goas = compute_outcomes(time_offset=time_offset)
    return goas


class RolloutThread(QtCore.QThread):
    finished = QtCore.pyqtSignal(object)
    pose = None
    orientation = None
    goal = None
    battery = None
    battery_rate = None
    velocity_rate = None
    time_offset = None
    known_obstacles = {}
    waypoints = []
    num_iterations = 10
    max_time = 60 * 10  # seconds
    wm_settings_path = '/data/webots/settings.yaml'
    wm_executable_path = '/home/cohrint-skynet/catkin_ws/src/etgoa_evaluation/world_model/worlds/rollout_simulation.wbt'

    def run(self):
        print('starting rollout thread')
        t1 = time.time()
        try:
            goas = do_rollout(self.pose, self.orientation, self.goal,
                              self.battery, self.battery_rate, self.velocity_rate,
                              self.known_obstacles, self.time_offset,
                              self.waypoints, self.max_time, self.num_iterations,
                              self.wm_settings_path, self.wm_executable_path)
            self.finished.emit(goas)
        except Exception as e:
            traceback.print_exc()
        t2 = time.time()
        print('exiting rollout thread. Secs: {:.2f}'.format(t2 - t1))

    def __str__(self):
        return ('pose: ' + str(self.pose) + '\n' +
                'orientation: ' + str(self.orientation) + '\n' +
                'goal: ' + str(self.goal) + '\n' +
                'battery: ' + str(self.battery) + '\n' +
                'battery_rate: ' + str(self.battery_rate) + '\n' +
                'velocity_rate: ' + str(self.velocity_rate) + '\n' +
                'time_offset: ' + str(self.time_offset) + '\n' +
                'known_obstacles: ' + str(self.known_obstacles) + '\n' +
                'waypoints: ' + str(self.waypoints) + '\n' +
                'num_iterations: ' + str(self.num_iterations) + '\n' +
                'max_time: ' + str(self.max_time) + '\n')


def example_rollout():
    pos = [0, 0, 0]
    orientation = [0, 0, 1, 0]
    goal = [10, 10]
    batt_level = 100
    batt_rate = 0.5
    vel_rate = 0.5
    max_time = 200
    iterations = 10
    known_obs = {}
    time_offset = 0.0
    waypoints = np.array([[0, 0], goal])
    do_rollout(pos, orientation, goal,
               batt_level, batt_rate, vel_rate,
               known_obs, time_offset,
               waypoints, max_time, iterations)


if __name__ == '__main__':
    example_rollout()
