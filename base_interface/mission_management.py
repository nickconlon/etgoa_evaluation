import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from motion_planning import rrt


class MissionManager:
    def __init__(self, mission_area_image_path):
        self.current_plan = None
        self.visualize = False
        self.mission_area_image = np.asarray(Image.open(mission_area_image_path))
        self.mission_area_bounds = np.array([[0, 800], [0, 800]])

    def update_plan(self, new_plan):
        self.current_plan = new_plan

    def delete_plan(self):
        self.current_plan = None

    def get_plan(self):
        return self.current_plan

    def has_plan(self):
        return self.current_plan is not None

    def plan_waypoints(self, robot_x, robot_y, goal_x, goal_y):
        # RRT goal = [y, x]
        goal = np.array([goal_y, goal_x])
        # RRT point [y, x]
        start = np.array([robot_y, robot_x])
        # RRT Obstacle
        obstacles = []
        # [ymin ymax], [xmin, xmax]
        bounds = self.mission_area_bounds
        plan = rrt.plan_rrt_webots(start, goal, obstacles, bounds,
                                   visualize_route=self.visualize,
                                   filename='')
        # Add back on the current position
        plan = np.vstack((np.array([robot_x, robot_y]), plan))
        self.update_plan(plan)

    def t_overlay_image(self):
        img = self.mission_area_image.copy()
        fig, ax = plt.subplots(frameon=False)
        self.visualize = False
        home = (501, 717)
        poi_b = (523, 151)
        poi_a = (402, 515)
        poi_c = (594, 360)
        poi_d = (685, 676)
        test_g = poi_c
        for s, g, color in zip([home, test_g], [test_g, home], ['black', 'red']):
            self.delete_plan()
            self.plan_waypoints(*s, *g)
            y, x, _ = img.shape
            plt.imshow(img)
            plt.xlim([0, x])
            plt.ylim([y, 0])
            p = self.current_plan
            p = np.vstack((s, p))
            plt.plot(p[:, 0], p[:, 1], '--', c=color, markersize=10)
            plt.scatter(p[:, 0], p[:, 1], c=color, s=10)
        plt.axis('off')
        plt.tight_layout()
        canvas = plt.gca().figure.canvas
        canvas.draw()
        data = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
        img = data.reshape(canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)
        return img

    def get_overlay_image(self, path_color='black'):
        img = self.mission_area_image.copy()
        fig, ax = plt.subplots(frameon=False)
        y, x, _ = img.shape
        plt.imshow(img)
        plt.xlim([0, x])
        plt.ylim([y, 0])
        if self.has_plan():
            p = self.current_plan
            plt.plot(p[:, 0], p[:, 1], '--', c=path_color, markersize=10)
            plt.scatter(p[:, 0], p[:, 1], c=path_color, s=10)
        plt.axis('off')
        plt.tight_layout()
        canvas = plt.gca().figure.canvas
        canvas.draw()
        data = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
        img = data.reshape(canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)
        return img


if __name__ == '__main__':
    m = MissionManager('../base_interface/mission_area.png')
    img = m.get_overlay_image()
    plt.imshow(img)
    plt.axis('off')
    plt.show()
