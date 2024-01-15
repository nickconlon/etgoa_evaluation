import traceback

import yaml
from motion_planning.rrt import Obstacle


class Settings:
    def __init__(self, settings_fname):
        self.fname = settings_fname
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

    def read(self):
        try:
            with open(self.fname, 'r') as file:
                settings = yaml.safe_load(file)
                for id, obs in settings['obstructions'].items():
                    ob = Obstacle(Obstacle.circle, obs[0], obs[1], id, data=obs[2])
                    self.obstructions.append(ob)

                for id, obs in settings['hazards'].items():
                    ob = Obstacle(Obstacle.circle, obs[0], obs[1], id, data=obs[2])
                    self.hazards.append(ob)

                for id, obs in settings['power_draws'].items():
                    ob = Obstacle(Obstacle.circle, obs[0], obs[1], id, data=obs[2])
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
        except Exception as e:
            traceback.print_exc()


if __name__ == '__main__':
    # [[x, y], [radius]]
    d = {
        'obstructions': {'o1': [[10, 15], [2], 1],
                         'o2': [[4, -5], [5], 1],
                         'o3': [[12, 21], [2.5], 1]
                         },
        'hazards': {'h1': [[-2, 5], [2.5], 0.25],  # [[x, y], [radius], [impact % change]]
                    },
        'power_draws': {'b1': [[-8, 0], [5], 1.5],
                        'b2': [[-12, 5], [5], 1.5]
                        },
        'record_path': './data',
        'map_path': './imgs/mission_area.png',
        'logo_path': './imgs/logo.png',
        'rollout_path': '/data/webots/rollout{}_state.npy',
        'condition': 'ET-GOA',  # TELEM, GOA, ET-GA
        'latitude_center': 40.01045433,
        'longitude_center': 105.24432153,
        'anomalies': False,
        'et_goa_threshold': 0.05,
        'et_goa_stds': [1.5, 1.5, 0.5, 1.5]
    }

    base = '../'
    print(yaml.dump(d))
    fname = base
    with open(fname + 'settings.yaml', 'w') as f:
        yaml.dump(d, f, default_flow_style=None, sort_keys=False)

    obs = Settings(fname + 'settings.yaml')
    obs.read()
    print(obs)
