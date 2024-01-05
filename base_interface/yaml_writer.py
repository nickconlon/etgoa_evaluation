import yaml
import motion_planning.rrt as rrt


def read_obstacles(yamlfile):
    obstacles = []
    with open(yamlfile, 'r') as file:
        settings = yaml.safe_load(file)
        for id, obs in settings['obstacles'].items():
            ob = rrt.Obstacle(rrt.Obstacle.circle, obs[0], obs[1], id)
            obstacles.append(ob)
    return obstacles


if __name__ == '__main__':
    # type = CIRCLE
    # center = ()
    # radius = []
    d = {
        'obstacles': {'o1': [[10, 15], [5]],
                      'o2': [[4, -5], [5]],
                      'o3': [[12, 21], [5]]
                      },
        'blockers': {'b1': [[10, 15], [5]]}
    }

    base = './'
    print(yaml.dump(d))
    fname = base
    with open(fname + 'obstacles.yaml', 'w') as f:
        yaml.dump(d, f, default_flow_style=None, sort_keys=False)

    obs = read_obstacles(fname + 'obstacles.yaml')
    [print(o) for o in obs]
