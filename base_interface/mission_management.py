import numpy as np
from motion_planning import rrt


class MissionManager:
    def __init__(self):
        self.current_plan = None
        self.visualize = True

    def update_plan(self, new_plan):
        self.current_plan = new_plan

    def delete_plan(self):
        self.current_plan = None

    def get_plan(self):
        return self.current_plan

    def has_plan(self):
        return len(self.current_plan) > 0 or self.current_plan is None

    def plan(self, robot_x, robot_y, goal_x, goal_y):
        # RRT goal = [y, x]
        goal = np.array([goal_y, goal_x])
        # RRT point [y, x]
        start = np.array([robot_y, robot_x])
        # RRT Obstacle
        obstacles = []
        # [ymin ymax], [xmin, xmax]
        bounds = np.array([[0, 100], [0, 100]])
        visualize = True
        plot_fname = '../data/route.png'
        plan = rrt.plan_rrt_webots(start, goal, obstacles, bounds,
                                   visualize_route=self.visualize,
                                   filename=plot_fname)
        self.update_plan(plan)


if __name__ == '__main__':
    m = MissionManager()
    m.plan(1, 1, 50, 50)
    print(m.has_plan())
    print(m.get_plan())
