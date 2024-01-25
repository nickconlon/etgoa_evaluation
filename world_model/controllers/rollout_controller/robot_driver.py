"""teleoperated_pioneer3at controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import yaml
from controller import Robot, Motor, Keyboard, Camera, Supervisor, Display
import numpy as np
import sys
import time
import pioneer_controller as pioneer
import matplotlib.pyplot as plt
from comms import settings_reader, StateObject
import json

TIME_STEP = 64

class Obstacle:
    circle = "CIRCLE"
    rectangle = "RECTANGLE"
    """
    Obstacles can be of type circle, or rectangle. Name should be unique but not enforced.
    circle: center (x,y), axis = [radius]
    rectangle: center (x,y), axis = [width, height]
    """
    def __init__(self, obs_type, obs_center, obs_axis, name, obs_angle=0, buffer=None):
        self.center = obs_center
        self.axis = obs_axis
        self.type = obs_type
        self.id = name
        self.angle = obs_angle
        self.buffer = buffer

class StateMachine:
    driving = 1
    done = 2
    replan = 3
    stopped = 4


def init_robot(robot):
    """
    Initialize the robot, return handles to its components.
    """
    keyboard = robot.getKeyboard()
    keyboard.enable(30)
    camera = robot.getDevice("camera")
    camera.enable(30)
    gps = robot.getDevice("gps")
    gps.enable(30)
    compass = robot.getDevice("compass")
    compass.enable(30)

    # setup the wheels
    wheels = []
    wheels.append(robot.getDevice('front left wheel'))
    wheels.append(robot.getDevice('front right wheel'))
    wheels.append(robot.getDevice('back left wheel'))
    wheels.append(robot.getDevice('back right wheel'))
    # set the target position of the motors
    for wheel in wheels:
        wheel.setPosition(float('inf'))

    return wheels, gps, compass, camera, keyboard


def get_bearing_degrees(north):
    """
    Get bearing given a vector pointing north
    """
    rad = np.arctan2(north[0], north[1])
    bearing = (rad - 1.5708) / np.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    return bearing


def goto(target, gps, compass, speed_noise=0.0):
    """
    return the wheel speeds necessary to move towards a target.
    """
    position = gps.getValues()
    y = target[1] - position[1]
    x = target[0] - position[0]

    direction = np.arctan2(y, x) * 180 / np.pi - 90 - get_bearing_degrees(compass.getValues())
    direction = direction % 360.0
    distance = np.linalg.norm(np.asarray([x, y]))

    speed = pioneer.stop()
    if abs(direction) > 2:
        if direction > 180:
            speed = pioneer.turn_right() + speed_noise
        else:
            speed = pioneer.turn_left() + speed_noise
    elif abs(distance) > 0.1:
        speed = pioneer.forward() + speed_noise

    if abs(distance) < 0.1:
        arrived = True
    else:
        arrived = False
    return speed, arrived


def remove_obstacles(_robot, _obstacle_names):
    for ob in _obstacle_names:
        pos = np.array(_robot.getFromDef(ob).getField('translation').getSFVec3f())
        _robot.getFromDef(ob).getField('translation').setSFVec3f([pos[0], pos[1], 1])


def randomize_obstacles(_robot, _obstacle_names):
    _minmaxx, _minmaxy, _minmaxz = [-2, 2], [-1, 1], [-0.25, 0.25]
    for ob in _obstacle_names:
        randx = 2
        while abs(randx - 2) < 0.5:
            randx = np.random.uniform(low=_minmaxx[0], high=_minmaxx[1])
        randy = 0
        while abs(randy - 0) < 0.5:
            randy = np.random.uniform(low=_minmaxy[0], high=_minmaxy[1])
        randz = 0.25  # np.random.uniform(low=_minmaxz[0], high=_minmaxz[1])
        _robot.getFromDef(ob).getField('translation').setSFVec3f([randx, randy, randz])
        print('Setting obstacle pos: ({:.2f}, {:.2f}, {:.2f})'.format(randx, randy, randz))

def set_obstacles(_robot, _obstacles):
    for ob, loc in _obstacles.items():
        _robot.getFromDef(ob).getField('translation').setSFVec3f([loc[0], loc[1], 0.0])

def update_known_obstacles(_robot, _obstacle_names, _fov=None):
    robot_position = np.array(_robot.getSelf().getField('translation').getSFVec3f())
    obstacles = []
    for ob in _obstacle_names:
        pos = np.array(_robot.getFromDef(ob).getField('translation').getSFVec3f())
        if pos[2] < -0.2:
            continue
        if _fov is None:
            obstacles.append(ob)
        elif np.linalg.norm(pos - robot_position) <= _fov:
            obstacles.append(ob)
    return obstacles


def get_obstacles(_robot, _obstacle_names, _fov=None):
    robot_position = np.array(_robot.getSelf().getField('translation').getSFVec3f())
    obstacles = []
    for ob in _obstacle_names:
        pos = np.array(_robot.getFromDef(ob).getField('translation').getSFVec3f())
        if pos[2] < -0.2 or pos[2] >= 1.0:
            continue
        if _fov is None:
            obstacles.append(Obstacle(Obstacle.rectangle, [pos[1], pos[0]], [1.2, 1.2], ob))
        elif np.linalg.norm(pos - robot_position) <= _fov:
            obstacles.append(Obstacle(Obstacle.rectangle, [pos[1], pos[0]], [1.2, 1.2], ob))
    return obstacles


def update_machine(_current_state_machine, _current_state, _goal, _next_waypoint, _max_time):
    _state_machine = StateMachine.driving
    if _current_state_machine == StateMachine.stopped:
        _state_machine = StateMachine.stopped
    elif len(_next_waypoint) == 0:  # Planning failed
        _state_machine = StateMachine.done
    elif _current_state.robot_time > _max_time:  # Ran out of time
        _state_machine = StateMachine.done
    elif np.linalg.norm(np.array([_current_state.pos[0], _current_state.pos[1]]) - _next_waypoint) < 0.1:  # The robot should replan
        _state_machine = StateMachine.replan
    return _state_machine


def run(goal, robot, wheels, gps, compass, known_obstacles, batt_level, batt_rate, vel_rate, waypoints, waypoint_index, run_number, run_prefix, max_time):
    velocity_noise = np.random.normal(loc=0.0, scale=0.5)
    battery = batt_level
    state_path = '/data/webots/{}{}_state.npy'.format(run_prefix, run_number)

    waypoint_counter = waypoint_index
    next_waypoint = waypoints[waypoint_counter]

    state = []
    t0 = robot.getTime()

    """
    Main loop
    """
    state_machine = StateMachine.driving
    while robot.step(TIME_STEP) != -1:
        """
        Capture the current state of the robot
        """
        sample_time = robot.getTime() - t0
        pose = robot.getSelf().getField('translation').getSFVec3f()
        orient = robot.getSelf().getField('rotation').getSFRotation()
        vel = robot.getSelf().getVelocity()
        battery = np.maximum(battery - 0.064 * batt_rate+(np.random.normal(0.0, 0.05)), 0.0)
        state_object = StateObject()
        state_object.set_state(pose, orient, vel[:2], battery, sample_time, time.time(), goal, waypoint_counter)
        state.append(np.array(state_object.get_state_array(), dtype=object))

        """
        Update the state machine
        """
        state_machine = update_machine(state_machine, state_object, goal, next_waypoint, max_time)

        """
        Update current waypoint, or update control actions, or count the task as complete
        """
        if state_machine == StateMachine.replan:
            waypoint_counter += 1
            if waypoint_counter >= len(waypoints):
                next_waypoint = []
            else:
                next_waypoint = waypoints[waypoint_counter]

        # Robot has more driving to do
        elif state_machine == StateMachine.driving:
            speed, arrived = goto(next_waypoint, gps, compass, velocity_noise)
            speed *= vel_rate  # adjust based on "real" speed
            '''
            # Sand trap functionality
            sand_pos = get_obstacles(robot, ['SAND'])
            if len(sand_pos) == 1:
                sand_pos = [2.4, 3.4]
                offset = 0.2
                robot_pos = robot.getSelf().getField('translation').getSFVec3f()
                if sand_pos[1] - offset < robot_pos[1] < sand_pos[1] + offset:
                    speed *= np.random.normal(loc=0.3, scale=0.5, size=4)
            '''
            for i, wheel in enumerate(wheels):
                wheel.setVelocity(speed[i])

        # Robot is done
        elif state_machine == StateMachine.done:
            speed = pioneer.stop()
            for i, wheel in enumerate(wheels):
                wheel.setVelocity(speed[i])
            np.save(state_path, state)
            break


def reset_robot(_robot, loc, rot):
    _robot.getSelf().getField('translation').setSFVec3f(loc)
    _robot.getSelf().getField('rotation').setSFRotation(rot)

