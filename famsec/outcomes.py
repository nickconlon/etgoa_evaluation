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
ori = [3, 4, 5, 6]  # [x, y, z, w]
speed = 7
obs = 8
gx = 9
gy = 10
t_robot = 11  # seconds since start
t_wall = 12  # seconds since epoch


def get_rollouts():
    base = '/data/webots/rollout{}_state.npy'
    data = []
    for i in range(5):
        d = np.load(base.format(i), allow_pickle=True)
        data.append(d)
    return data


def outcome_time(rollouts, outcome):
    """
    time: the mission time at the final state
    GOA < 10 minutes

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
    b_t+1 = b_t-v * eff
    GOA > 50%

    :param rollouts:
    :return:
    """
    batt = []
    for rollout in rollouts:
        batt.append(1)
    return batt


def outcome_obstacles(rollouts, outcome):
    """
    stability: the mean obstacle avoidance ability of the robot
    s_t = 1-p(obstacle)_t, p(obstacle) ~ N(mu, sigma)
    GOA > 90%

    :param obstacles:
    :param rollouts:
    :return:
    """
    quality = []
    for rollout in rollouts:
        goal = rollout[-1, gx:gy + 1]
        pos = rollout[-1, px:py + 1]
        d = 1  # np.linalg.norm(pos)
        quality.append(d)
    return quality


def outcome_survey_quality(rollouts, outcome):
    """
    survey quality: the closest the robot was able to get to the POI
    sq = min d(robot, poi)
    GOA > 90

    :param goal:
    :param rollouts:
    :return:
    """
    quality = []
    for rollout in rollouts:
        goal = rollout[-1, gx:gy + 1]
        pos = rollout[-1, px:py + 1]
        d = np.linalg.norm(pos - goal)
        if d < 1.0:
            quality.append(1)
        else:
            quality.append(0)
    return quality


def outcome_arrival(rollouts, outcome):
    """
    arrival: if the robot arrived at the goal location
    GOA = 1

    :param rollouts:
    :param goal:
    :return:
    """
    arrived = []
    for rollout in rollouts:
        goal = rollout[-1, gx:gy + 1]
        pos = rollout[-1, px:py + 1]
        d = np.linalg.norm(pos - goal)
        if d < 1.0:
            arrived.append(1)
        else:
            arrived.append(0)
    return arrived


def compute_outcomes():
    print('computing outcome')
    data = get_rollouts()
    goas = {}
    functions = {
        'arrival': [outcome_arrival, 1, [-0.5, 0.5, 1.5], 2],
        'battery': [outcome_battery, 1, [-0.5, 0.5, 1.5], 2],
        'obstacles': [outcome_obstacles, 1, [-0.5, 0.5, 1.5], 2],
        'time': [outcome_time, 100, [-0.5, 0.5, 1.5], 2],
        'quality': [outcome_survey_quality, 1, [-0.5, 0.5, 1.5], 2],
    }

    for k, v in functions.items():
        f = v[0]
        o = v[1]
        bins = v[2]
        zstar = v[3]
        outcomes = f(data, o)
        goa = fmc.assess_rollouts(outcomes, bins, zstar)
        goas[k] = goa
    print(goas)
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
