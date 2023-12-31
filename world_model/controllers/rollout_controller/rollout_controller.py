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

    start = settings['start_position']
    orientation = settings['start_orientation']
    goal = np.array(settings['goal_position'])
    waypoint_index = settings['waypoint_index']
    num_runs = settings['num_iterations']
    run_type = settings['prefix']
    known_obs = settings['known_obs']
    max_time = settings['max_time']
    wpx = settings['wp_x']
    wpy = settings['wp_y']
    waypoints = np.dstack((wpx, wpy)).squeeze()

    print('start', start)
    print('orint', orientation)
    print('goal', goal)
    print('wps', waypoints)

    reset_robot(robot, start, orientation)
    set_obstacles(robot, known_obs)
    all_obstacles = ['BOX1', 'BOX2', 'BOX3', 'BOX4', 'SAND', 'WALL']
    
    obstacles_pos = get_obstacles(robot, known_obs)
    
    a = set(all_obstacles)
    b = set(known_obs)
    remove_obstacles(robot, b.symmetric_difference(a))

    # Real run first, so we don't have to play games resetting obstacles
    t0 = time.time()
    for run_id in range(num_runs):
        print('run {}'.format(run_id))
        run(goal, robot, wheels, gps, compass, known_obs, waypoints, waypoint_index, run_id, run_type, max_time)
        print(orientation)
        reset_robot(robot, start, orientation)
    t1 = time.time()
    print('rollout time: {:.3f}'.format(t1 - t0))
    print('robot time: {:.3f}'.format(robot.getTime()))
    robot.simulationQuit(0)
