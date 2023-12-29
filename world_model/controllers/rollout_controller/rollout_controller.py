"""teleoperated_pioneer3at controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Keyboard, Camera, Supervisor, Display
import numpy as np
import time
import sys
from comms import settings_reader
from robot_driver import reset_robot, init_robot, get_obstacles, set_obstacles, remove_obstacles, run

if __name__ == '__main__':
    settings = settings_reader('settings.yaml')
    robot = Supervisor()
    wheels, gps, compass, camera, keyboard = init_robot(robot)
    run_ids = np.arange(0, 10)

    start = settings['start_position']
    orientation = settings['start_orientation']
    goal = np.array(settings['goal_position'])
    waypoint_index = settings['waypoint_index']
    should_pub = settings['publish']
    num_runs = settings['num_iterations']
    run_type = settings['prefix']
    known_obs = settings['known_obs']
    
    print('start', start)
    print('orint', orientation)
    print('goal', goal)
    
    reset_robot(robot, start, orientation)
    set_obstacles(robot, known_obs)
    max_time = 140
    all_obstacles = ['BOX1', 'BOX2', 'BOX3', 'BOX4', 'SAND', 'WALL']
    if 'wp_x' in settings:
        wpx = settings['wp_x']
        wpy = settings['wp_y']
        waypoints = np.dstack((wpx, wpy)).squeeze()
    else:
        # waypoints = plan(goal, robot.getSelf().getField('translation').getSFVec3f(), obstacles_pos)
        waypoints = [[-5.71397894, -1.01152951],
                     [-5.00592259, -1.02219796],
                     [-3.95644449, -1.08216215],
                     [-2.90920252, -0.99114424],
                     [-1.86991786, -0.83338681],
                     [-0.8239004, -0.72923559],
                     [0.21703279, -0.58274984],
                     [1.26614234, -0.64884942],
                     [2.30934624, -0.51952182],
                     [3.35255014, -0.39019422],
                     [4.87977129, 0.67226107],
                     [5.85070225, 0.26940493],
                     [6.5, 0.]]

    print('wps', waypoints)
    
    obstacles_pos = get_obstacles(robot, known_obs)
    
    a = set(all_obstacles)
    b = set(known_obs)
    remove_obstacles(robot, b.symmetric_difference(a))

    # Real run first, so we don't have to play games resetting obstacles
    t0 = time.time()
    for run_id in range(num_runs):
        print('run {}'.format(run_id))
        run(goal, robot, wheels, gps, compass, known_obs, waypoints, waypoint_index, run_id, should_pub, run_type, max_time)
        print(orientation)
        reset_robot(robot, start, orientation)
    t1 = time.time()
    print('rollout time: {:.3f}'.format(t1 - t0))
    print('robot time: {:.3f}'.format(robot.getTime()))
    robot.simulationQuit(0)
