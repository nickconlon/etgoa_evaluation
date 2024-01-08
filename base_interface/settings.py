import traceback

import yaml
import motion_planning.rrt as rrt


class Settings:
    def __init__(self, settings_fname):
        self.fname = settings_fname
        self.obstacles = []
        self.blockers = []
        self.record_path = None
        self.map_path = None
        self.logo_path = None
        self.rollout_path = None
        self.condition = None
        self.lat_center = None
        self.lon_center = None
        self.anomalies = False
        self.et_goa_threshold = None

    def read(self):
        try:
            with open(self.fname, 'r') as file:
                settings = yaml.safe_load(file)
                for id, obs in settings['obstacles'].items():
                    ob = rrt.Obstacle(rrt.Obstacle.circle, obs[0], obs[1], id)
                    self.obstacles.append(ob)
                for id, obs in settings['blockers'].items():
                    ob = rrt.Obstacle(rrt.Obstacle.circle, obs[0], obs[1], id)
                    self.blockers.append(ob)
                self.record_path = settings['record_path']
                self.map_path = settings['map_path']
                self.logo_path = settings['logo_path']
                self.rollout_path = settings['rollout_path']
                self.condition = settings['condition']
                self.lat_center = settings['latitude_center']
                self.lon_center = settings['longitude_center']
                self.anomalies = settings['anomalies']
                self.et_goa_threshold = settings['et-goa_threshold']
        except Exception as e:
            traceback.print_exc()



if __name__ == '__main__':
    d = {
        'obstacles': {'o1': [[10, 15], [5]],  # [[x, y], [radius]]
                      'o2': [[4, -5], [5]],
                      'o3': [[12, 21], [5]]
                      },
        'blockers': {'b1': [[10, 15], [5]]},  # [[x, y], [radius]]
        'record_path': './data/recording{}.csv',
        'map_path': './imgs/mission_area.png',
        'logo_path': './imgs/logo.png',
        'rollout_path': '/data/webots/rollout{}_state.npy',
        'condition': 'ET-GOA',  # TELEM, GOA, ET-GA
        'latitude_center': 40.01045433,
        'longitude_center': 105.24432153,
        'anomalies': False,
        'et-goa_threshold': 0.05
    }

    base = '../'
    print(yaml.dump(d))
    fname = base
    with open(fname + 'settings.yaml', 'w') as f:
        yaml.dump(d, f, default_flow_style=None, sort_keys=False)

    obs = Settings(fname + 'settings.yaml')
    obs.read()
    print(obs)
