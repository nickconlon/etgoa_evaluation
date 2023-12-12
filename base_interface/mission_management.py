import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

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
        bounds = np.array([[0, 800], [0, 800]])
        visualize = True
        plot_fname = '../data/route.png'
        plan = rrt.plan_rrt_webots(start, goal, obstacles, bounds,
                                   visualize_route=self.visualize,
                                   filename=plot_fname)
        self.update_plan(plan)

    def overlay_image(self, img):
        self.visualize = False
        home = (501, 717)
        poi_b = (523, 151)
        poi_a = (402, 515)
        poi_c = (594, 360)
        poi_d = (685, 676)
        test_g = poi_c
        for s, g, color in zip([home, test_g], [test_g, home], ['black', 'red']):
            self.delete_plan()
            m.plan(*s, *g)
            y, x, _ = img.shape
            plt.imshow(img)
            plt.xlim([0, x])
            plt.ylim([y, 0])
            p = self.current_plan
            p = np.vstack((s, p))
            plt.plot(p[:, 0], p[:, 1], '--', c=color, markersize=10)
            plt.scatter(p[:, 0], p[:, 1], c=color, s=10)
        plt.show()


if __name__ == '__main__':
    m = MissionManager()
    m.overlay_image(np.array(Image.open('../base_interface/mission_area.png')))
