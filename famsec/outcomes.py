import matplotlib.pyplot as plt
import numpy as np
import famsec.factorized_machine_self_confidence as fmc

# s = [
#       x, y, z,    > position of robot (meters)
#       speed,      > robot speed (meters/second)
#       heading,    > orientation (degrees from N)
#       battery,    > 0-100 (percent)
#       stability,  > 0-100 (percent)
#       quality,    > 0-100 (percent)
#       gx, gy,     > position of POI/goal (meters)
#       time        > mission time (seconds)
#       ]
#
px = 0
py = 1
pz = 2
ori_x = 3  # [x, y, z, w]
ori_y = 4  # [x, y, z, w]
ori_z = 5  # [x, y, z, w]
ori_w = 6  # [x, y, z, w]
speed = 7
obs = 8
goal_x = 9
goal_y = 10
t_robot = 11  # seconds since start
t_wall = 12  # seconds since epoch


def get_rollouts():
    base = '/data/webots/rollout{}_state.npy'
    data = []
    for i in range(10):
        d = np.load(base.format(i), allow_pickle=True)
        data.append(d)
    return data


def outcome_time(rollouts, outcome):
    """
    time: the mission time at the final state
    GOA = final time < outcome

    :param outcome:
    :param rollouts:
    :return:
    """
    times = []
    for rollout in rollouts:
        tt = rollout[-1, t_robot]
        if tt < outcome:
            times.append(1)
        else:
            times.append(0)
    return times


def outcome_battery(rollouts, outcome):
    """
    battery level: the battery level at the final state
    GOA  == final battery > outcome

    :param outcome:
    :param rollouts:
    :return:
    """
    batt = []
    for rollout in rollouts:
        a = rollout[:, 8] >= outcome
        a = int(a.sum() == len(rollout))
        batt.append(a)
    return batt


def outcome_obstacles(rollouts, outcome):
    """
    stability: the mean obstacle avoidance ability of the robot
    GOA = obstacles hit < outcome

    :param outcome:
    :param rollouts:
    :return:
    """
    quality = []
    for rollout in rollouts:
        goal = rollout[-1, goal_x:goal_y + 1]
        pos = rollout[-1, px:py + 1]
        d = 1  # np.linalg.norm(pos)
        quality.append(d)
    return quality


def outcome_survey_quality(rollouts, outcome):
    """
    survey quality: the closest the robot was able to get to the POI
    GOA = total quality > outcome

    :param outcome:
    :param rollouts:
    :return:
    """
    quality = []
    for rollout in rollouts:
        goal = rollout[-1, goal_x:goal_y + 1]
        pos = rollout[-1, px:py + 1]
        d = np.linalg.norm(pos - goal)
        if d < 1.0:
            quality.append(1)
        else:
            quality.append(0)
    return quality


def outcome_poi_arrival(rollouts, outcome):
    """
    arrival: if the robot arrived at the goal location
    GOA = |robot-goal| < outcome

    :param outcome:
    :param rollouts:
    :return:
    """
    arrived = []
    for rollout in rollouts:
        goal = rollout[-1, goal_x:goal_y + 1]
        pos = rollout[-1, px:py + 1]
        d = np.linalg.norm(pos - goal)
        if d < 2.0:
            arrived.append(1)
        else:
            arrived.append(0)
    return arrived


def outcome_home_arrival(rollouts, outcome):
    """
    arrival: if the robot arrived at the home location
    GOA = |robot-home| < outcome

    :param outcome:
    :param rollouts:
    :return:
    """
    arrived = []
    for rollout in rollouts:
        goal = rollout[-1, goal_x:goal_y + 1]
        pos = rollout[-1, px:py + 1]
        d = np.linalg.norm(pos - goal)
        if d < 2.0:
            arrived.append(1)
        else:
            arrived.append(0)
    return arrived

def compute_outcomes(time_offset=0):
    print('computing outcome')
    data = get_rollouts()
    for d in data:
        d[:, 11] += time_offset
    goas = {}
    functions = {
        'poi_arrival': [outcome_poi_arrival, 1, [-0.5, 0.5, 1.5], 2],
        'home_arrival': [outcome_home_arrival, 1, [-0.5, 0.5, 1.5], 2],
        'battery': [outcome_battery, 20, [-0.5, 0.5, 1.5], 2],
        'obstacles': [outcome_obstacles, 1, [-0.5, 0.5, 1.5], 2],
        'time': [outcome_time, 60*10, [-0.5, 0.5, 1.5], 2],
    }

    for k, v in functions.items():
        f = v[0]
        o = v[1]
        bins = v[2]
        zstar = v[3]
        outcomes = f(data, o)
        goa = fmc.assess_rollouts(outcomes, bins, zstar)
        goas[k] = goa
    return goas


if __name__ == '__main__':
    outcomes = compute_outcomes()
    print(outcomes)
    '''
    data = get_rollouts()
    outcome = 141
    outcomes = outcome_time(data, outcome)
    assmt = fmc.assess_rollouts(outcomes, [-0.5, 0.5, 1.5], z_star=2)
    plt.title('time < {}: {:.2f}'.format(135, assmt))
    plt.hist(outcomes, bins=[-0.5, 0.5, 1.5, 2.5])
    plt.show()

    outcomes = outcome_arrival(data, 1)
    outcome = 141
    assmt = fmc.assess_rollouts(outcomes, [-0.5, 0.5, 1.5], z_star=2)
    plt.title('arrival: {:.2f}'.format(assmt))
    plt.hist(outcomes, bins=[-0.5, 0.5, 1.5, 2.5])
    plt.show()

    outcomes = outcome_survey_quality(data, 1)
    assmt = fmc.assess_rollouts(outcomes, [-0.5, 0.5, 1.5], z_star=2)
    plt.title('quality > 0.9: {:.2f}'.format(assmt))
    plt.hist(outcomes, bins=[-0.5, 0.5, 1.5, 2.5])
    plt.show()

    outcomes = outcome_obstacles(data, 1)
    assmt = fmc.assess_rollouts(outcomes, [-0.5, 0.5, 1.5], z_star=2)
    plt.title('stability > 0.9: {:.2f}'.format(assmt))
    plt.hist(outcomes, bins=[-0.5, 0.5, 1.5, 2.5])
    plt.show()
    '''
