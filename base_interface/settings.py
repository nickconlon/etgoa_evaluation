import traceback

import yaml
from motion_planning.rrt import Obstacle
from motion_planning.projections import PointOfInterest

class Settings:
    def __init__(self, settings_fname):
        self.fname = settings_fname
        self.pois = []
        self.obstructions = []
        self.hazards = []
        self.power_draws = []
        self.record_path = None
        self.map_path = None
        self.logo_path = None
        self.rollout_path = None
        self.condition = None
        self.lat_center = None
        self.lon_center = None
        self.anomalies = False
        self.et_goa_threshold = None
        self.et_goa_stds = None
        self.batt_drain_anomaly = None
        self.num_backup_batteries = None
        self.speed_anomaly = None
        self.available_pois = None
        self.show_surveys = None
        self.area = None
        self.robot_name = None
        self.mode = None
        self.training = None
        self.map = None

    def read(self):
        try:
            with open(self.fname, 'r') as file:
                settings = yaml.safe_load(file)

                for id, poi in settings['pois'].items():
                    p = PointOfInterest(*poi['center'], name=id)
                    self.pois.append(p)

                for id, obs in settings['obstructions'].items():
                    ob = Obstacle(Obstacle.circle,
                                  obs['center'],
                                  [obs['radius']],
                                  id,
                                  data=obs['effect'],
                                  visible=obs['visible'])
                    self.obstructions.append(ob)

                for id, obs in settings['hazards'].items():
                    ob = Obstacle(Obstacle.circle,
                                  obs['center'],
                                  [obs['radius']],
                                  id,
                                  data=obs['effect'],
                                  visible=obs['visible'])
                    self.hazards.append(ob)

                for id, obs in settings['power_draws'].items():
                    ob = Obstacle(Obstacle.circle,
                                  obs['center'],
                                  [obs['radius']],
                                  id,
                                  data=obs['effect'],
                                  visible=obs['visible'])
                    self.power_draws.append(ob)

                self.record_path = settings['record_path']
                self.map_path = settings['map_path']
                self.logo_path = settings['logo_path']
                self.rollout_path = settings['rollout_path']
                self.condition = settings['condition']
                self.lat_center = settings['latitude_center']
                self.lon_center = settings['longitude_center']
                self.anomalies = settings['anomalies']
                self.et_goa_threshold = settings['et_goa_threshold']
                self.et_goa_stds = settings['et_goa_stds']
                self.batt_drain_anomaly = settings['batt_drain_anomaly']
                self.num_backup_batteries = settings['num_backup_batteries']
                self.speed_anomaly = settings['speed_anomaly']
                self.available_pois = settings['available_pois']
                self.show_surveys = settings['show_surveys']
                self.area = settings['area']
                self.robot_name = settings['robot_name']
                self.mode = settings['mode']
                self.training = settings['training']
                self.map = settings['map']

        except Exception as e:
            traceback.print_exc()


def create():
    """
    Create a settings file

    obstacles:          [[x, y], [radius], impact % change]
    et_goa_stds:        [min std dev s1, s2, s3]
    et_goa_threshold:   ET-GOA threshold
    batt_drain_rate     % per second
    """
    d = {
        'pois':{'A': {'center': [1, 9]},
                'B': {'center': [4, 7]},
                'H': {'center': [2.5, 0]}
                },
        'obstructions': {
            'o1': {'center': [4, 5], 'radius': 1, 'effect': 1, 'visible':True}, #{'center': [], 'radius': r, 'effect': e, 'visible': T/F,'active': T/F}
            'o2': {'center': [1, 7], 'radius': 1.2, 'effect': 1, 'visible':True},
            'o3': {'center': [-0.5, 6.5], 'radius': 1, 'effect': 1, 'visible':True}
                         },
        'hazards': {
            'h1': {'center': [5, 3], 'radius': 1, 'effect': 0.005, 'visible':True},
                    },
        'power_draws': {
            'b1': {'center': [2, 5], 'radius': 1, 'effect': 5.0, 'visible':True}
                        },
        'area': 'gazebo',
        'mode': 'cu',
        'robot_name': 'case',
        'record_path': './data',
        'map_path': './imgs/display_area.png',
        'logo_path': './imgs/logo.png',
        'rollout_path': '/data/webots/rollout{}_state.npy',
        'condition': 'ET-GOA',  # TELEM, GOA, ET-GOA
        'latitude_center': 40.01045433,
        'longitude_center': 105.24432153,
        'anomalies': False,
        'et_goa_threshold': 0.05,
        'et_goa_stds': [1.5, 1.5, 0.5, 1.5],
        'batt_drain_anomaly': 0.05,  # % change in battery drain rate
        'num_backup_batteries': 10,
        'speed_anomaly': 0.25,  # % change in speed
        'available_pois': ['A', 'B', 'C', 'D'],
        'show_surveys':  [True, True, True, True], #baseline trust, post planning trust, post execution trust, usability
        'training': False,
        'map': 'm1'
    }

    print(yaml.dump(d))
    with open('../settings.yaml', 'w') as f:
        yaml.dump(d, f, default_flow_style=None, sort_keys=False)


if __name__ == '__main__':
    create()
    obs = Settings('../settings.yaml')
    obs.read()
    print(obs)
