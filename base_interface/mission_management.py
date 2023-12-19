import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from motion_planning import rrt
from motion_planning import projections


class MissionManager:
    def __init__(self, mission_area_image_path):
        self.home = (501, 717)
        self.poi_b = (523, 151)
        self.poi_a = (402, 515)
        self.poi_c = (594, 360)
        self.poi_d = (685, 676)

        self.current_plan = None
        self.visualize = False
        self.mission_area_image = np.asarray(Image.open(mission_area_image_path))
        self.mission_area_bounds = np.array([[-50, 50], [-50, 50]])

    def update_plan(self, new_plan):
        self.current_plan = new_plan

    def delete_plan(self):
        self.current_plan = None

    def get_plan(self):
        return self.current_plan

    def has_plan(self):
        return self.current_plan is not None

    def plan_known_poi(self, poi_string):
        poi = None
        if poi_string == 'POI A':
            poi = self.poi_a
        elif poi_string == 'POI B':
            poi = self.poi_b
        elif poi_string == 'POI C':
            poi = self.poi_c
        elif poi_string == 'POI D':
            poi = self.poi_d
        else:
            self.delete_plan()
        if poi:
            self.plan_waypoints(*self.home, *poi)

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

        test_g = self.poi_c
        for s, g, color in zip([self.home, test_g], [test_g, self.home], ['black', 'red']):
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
    lat_center = 40.01045433
    lon_center = 105.24432153

    pp = projections.Projector(lat_center, lon_center)
    pp.setup()
    pois = pp.get_pois()
    home = pois[-1]
    tgt = pois[1]

    m = MissionManager('../base_interface/mission_area.png')
    m.plan_waypoints(home.x, home.y, tgt.x, tgt.y)
    c = m.current_plan
    cc = []
    for p in c:
        px, py = pp.cartesian_to_pixel(p[0], p[1])
        cc.append(np.array([px, py]))
    m.current_plan = np.array(cc)
    img = m.get_overlay_image()
    plt.imshow(img)
    plt.axis('off')
    plt.show()
