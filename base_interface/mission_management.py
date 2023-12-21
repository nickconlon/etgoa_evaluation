import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from motion_planning import rrt
from motion_planning import projections


class MissionManager:
    def __init__(self, mission_area_image_path):
        lat_center, lon_center = 40.01045433, 105.24432153
        area_miny, area_maxy, area_minx, area_maxx = -50, 50, -50, 50
        self.projector = projections.Projector(lat_center, lon_center)
        self.projector.setup()
        pois = self.projector.get_pois()
        self.poi_a = pois[0]
        self.poi_b = pois[1]
        self.poi_c = pois[2]
        self.poi_d = pois[3]
        self.home = pois[4]

        self.current_plan = None
        self.visualize = False
        self.mission_area_image = np.asarray(Image.open(mission_area_image_path))
        self.mission_area_bounds = np.array([[area_miny, area_maxy], [area_minx, area_maxx]])

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
            self.plan_waypoints(self.home.x, self.home.y, poi.x, poi.y)

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

    def get_overlay_image(self, path_color='black'):
        img = self.mission_area_image.copy()
        fig, ax = plt.subplots(frameon=False)
        y, x, _ = img.shape
        plt.imshow(img)
        plt.xlim([0, x])
        plt.ylim([y, 0])
        if self.has_plan():
            plan = self.current_plan
            pixel_plan = []
            for point in plan:
                pixel_plan.append(self.projector.cartesian_to_pixel(point[0], point[1]))
            pixel_plan = np.array(pixel_plan)
            plt.plot(pixel_plan[:, 0], pixel_plan[:, 1], '--', c=path_color, markersize=10)
            plt.scatter(pixel_plan[:, 0], pixel_plan[:, 1], c=path_color, s=10)
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
    m.plan_waypoints(m.home.x, m.home.y, m.poi_c.x, m.poi_c.y)
    img = m.get_overlay_image()
    plt.imshow(img)
    plt.axis('off')
    plt.show()
