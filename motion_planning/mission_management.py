import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from PIL import Image

from motion_planning import rrt


class MissionManager:
    ASPEN = 'aspen'
    OUTDOOR = 'outdoor'

    def __init__(self, mission_area_image_path, projector, obstructions, hazards, power_draws):
        area_miny, area_maxy, area_minx, area_maxx = -20, 40, -20, 20
        self.display_bounds = [-50, 50, -50, 50]  # minx, maxx, miny, maxy
        self.projector = projector

        pois = self.projector.get_pois()
        self.poi_a = pois[0]
        self.poi_b = pois[1]
        self.poi_c = pois[2]
        self.poi_d = pois[3]
        self.home = pois[4]
        self.zero = pois[5]
        self.location = self.ASPEN
        self.current_plan = None
        self.current_goal = None
        self.captured_goal = False
        self.captured_home = False
        self.all_obstacles = {}
        self.active_obstacles = set()
        self.visualize = False
        self.mission_area_image = np.asarray(Image.open(mission_area_image_path))
        self.mission_area_bounds = np.array([[area_minx, area_maxx], [area_miny, area_maxy]])
        self.obstructions = obstructions
        self.hazards = hazards
        self.power_draws = power_draws
        self.setup_obstacles(obstructions)
        self.setup_obstacles(power_draws)
        self.setup_obstacles(hazards)
        self.activate_obstacles([o.id for o in hazards])
        self.activate_obstacles([o.id for o in power_draws])

    def setup_obstacles(self, obstacles):
        for o in obstacles:
            if o.id not in self.all_obstacles:
                print('adding ', o.id)
                self.all_obstacles[o.id] = o
            else:
                print('found duplicate obstacle ', o.id)

    def activate_obstacles(self, ob_ids):
        print('activating obstacles', ob_ids)
        for ob_id in ob_ids:
            if ob_id in self.all_obstacles:
                self.active_obstacles.add(ob_id)
            else:
                print('Cannot find obstacle to activate ', ob_id)

    def deactivate_obstacles(self, ob_ids):
        print('deactivating obstacles', ob_ids)
        for ob_id in ob_ids:
            if ob_id in self.active_obstacles:
                self.active_obstacles.remove(ob_id)
            else:
                print('Cannot find obstacle to deactivate ', ob_id)

    def get_active_obstacles(self):
        active = []
        for ob_id in self.active_obstacles:
            if ob_id in self.all_obstacles:
                active.append(self.all_obstacles[ob_id])
        return active

    def update_plan(self, new_plan):
        self.current_plan = new_plan

    def delete_plan(self):
        self.current_plan = None
        self.current_goal = None

    def get_plan(self):
        return self.current_plan

    def has_plan(self):
        return self.current_plan is not None

    def plan_known_poi(self, robot_x, robot_y, poi_string, tofrom=False):
        poi = None
        if poi_string == 'POI A':
            poi = self.poi_a
        elif poi_string == 'POI B':
            poi = self.poi_b
        elif poi_string == 'POI C':
            poi = self.poi_c
        elif poi_string == 'POI D':
            poi = self.poi_d
        elif poi_string == 'HOME':
            poi = self.home
        else:
            self.delete_plan()
        if poi:
            if tofrom:
                self.plan_to_from(robot_x, robot_y, poi.x, poi.y, self.home.x, self.home.y)
            else:
                self.plan_waypoints(robot_x, robot_y, poi.x, poi.y)
        self.current_goal = poi
        self.captured_goal = False
        self.captured_home = False

    def plan_waypoints(self, robot_x, robot_y, goal_x, goal_y):
        # RRT goal = [y, x]
        goal = np.array([goal_x, goal_y])
        # RRT point [y, x]
        start = np.array([robot_x, robot_y])
        # RRT Obstacle
        obstacles = self.obstructions
        # [ymin ymax], [xmin, xmax]
        bounds = self.mission_area_bounds
        plan = rrt.plan_rrt_webots(start, goal, obstacles, bounds,
                                   visualize_route=self.visualize,
                                   filename='')
        # Add back on the current position
        plan = np.vstack((np.array([robot_x, robot_y]), plan))
        self.update_plan(plan)

    def plan_to_from(self, robot_x, robot_y, goal_x, goal_y, home_x, home_y):
        # RRT goal = [y, x]
        goal = np.array([goal_x, goal_y])
        robot = np.array([robot_x, robot_y])
        home = np.array([home_x, home_y])
        obstacles = self.obstructions
        bounds = self.mission_area_bounds
        # from robot (x, y) to goal (x, y)
        plan_to = rrt.plan_rrt_webots(robot, goal, obstacles, bounds,
                                      visualize_route=self.visualize,
                                      filename='')
        # from goal (x, y) to home (x, y)
        plan_from = rrt.plan_rrt_webots(goal, home, obstacles, bounds,
                                        visualize_route=self.visualize,
                                        filename='')

        if len(plan_to) == 0 or len(plan_from) == 0:
            print('Planning failed')
        else:
            plan_to = np.vstack((np.array([robot_x, robot_y]), plan_to))
            plan = np.vstack((plan_to, plan_from))

            self.update_plan(plan)

    def get_overlay_image_aspen(self, robot_x, robot_y, path_color='black'):
        fig, ax = plt.subplots(frameon=False, figsize=(6, 6))
        ax.imshow(self.mission_area_image, extent=self.display_bounds)
        mission_area_corner = (self.mission_area_bounds[0][0], self.mission_area_bounds[1][0])
        mission_area_width = self.mission_area_bounds[0][1]-self.mission_area_bounds[0][0]
        mission_area_height = self.mission_area_bounds[1][1]-self.mission_area_bounds[1][0]
        patch = Rectangle(mission_area_corner, mission_area_width, mission_area_height, facecolor='none', edgecolor='black', linewidth=5)
        ax.add_patch(patch)
        patch = Rectangle(mission_area_corner, mission_area_width, mission_area_height, facecolor='none', edgecolor='red', linestyle='--', linewidth=2)
        ax.add_patch(patch)
        ax.set_xlim(self.display_bounds[:2])
        ax.set_ylim(self.display_bounds[2:])

        # plot the POIs
        for poi in [self.poi_a, self.poi_b, self.poi_c, self.poi_d]:
            ax.add_patch(Circle((poi.x, poi.y), radius=2.5, facecolor='green', edgecolor='black'))
            ax.annotate(poi.name, (poi.x, poi.y), size='large', va='center', ha='center')

        ax.scatter([self.home.x], [self.home.y], c='gold', s=200, marker='*')
        ax.annotate(self.home.name, (self.home.x+4, self.home.y), size='large', va='center', ha='center')

        # plot the obstacles
        for o in self.obstructions:
            rx, ry = o.center
            c = plt.Circle((rx, ry), radius=o.axis[0], edgecolor='red', facecolor='red', alpha=0.5)
            ax.add_patch(c)

        for o in self.hazards:
            rx, ry = o.center
            c = plt.Circle((rx, ry), radius=o.axis[0], edgecolor='orange', facecolor='orange', alpha=0.5)
            ax.add_patch(c)

        for o in self.power_draws:
            rx, ry = o.center
            c = plt.Circle((rx, ry), radius=o.axis[0], edgecolor='blue', facecolor='blue', alpha=0.5)
            ax.add_patch(c)

        # plot the plan
        if self.has_plan():
            plan = self.current_plan
            pixel_plan = [[robot_x, robot_y]]
            for point in plan:
                pixel_plan.append([point[0], point[1]])
            pixel_plan = np.array(pixel_plan)
            ax.plot(pixel_plan[:, 0], pixel_plan[:, 1], '--', c=path_color, markersize=10)
            ax.scatter(pixel_plan[:, 0], pixel_plan[:, 1], c=path_color, s=10)

        # plot the robot
        c = plt.Circle((robot_x, robot_y), radius=1, edgecolor='blue', facecolor='blue')
        ax.add_patch(c)

        plt.grid()
        ax.axis('square')
        canvas = plt.gca().figure.canvas
        canvas.draw()
        data = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
        img = data.reshape(canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)
        return img

    def get_overlay_image_outdoor(self, robot_x, robot_y, path_color='black', ):
        img = self.mission_area_image.copy()
        fig, ax = plt.subplots(frameon=False)
        y, x, _ = img.shape
        plt.imshow(img)
        plt.xlim([0, x])
        plt.ylim([y, 0])
        if self.has_plan():
            plan = self.current_plan
            pixel_plan = [self.projector.cartesian_to_pixel(robot_x, robot_y)]
            for point in plan:
                pixel_plan.append(self.projector.cartesian_to_pixel(point[0], point[1]))
            pixel_plan = np.array(pixel_plan)
            plt.plot(pixel_plan[:, 0], pixel_plan[:, 1], '--', c=path_color, markersize=10)
            plt.scatter(pixel_plan[:, 0], pixel_plan[:, 1], c=path_color, s=10)

        rx, ry = self.projector.cartesian_to_pixel(robot_x, robot_y)
        plt.scatter([rx], [ry], c='blue', s=100)
        plt.axis('off')
        plt.axis('equal')
        plt.tight_layout()
        canvas = plt.gca().figure.canvas
        canvas.draw()
        data = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
        img = data.reshape(canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)
        return img

    def get_overlay_image(self, robot_x, robot_y, path_color='black'):
        if self.location == self.ASPEN:
            return self.get_overlay_image_aspen(robot_x, robot_y, path_color)
        elif self.location == self.OUTDOOR:
            return self.get_overlay_image_outdoor(robot_x, robot_y, path_color)

    def update_progress(self, robot_x, robot_y):
        # if [rx, ry]-[px, py] < d -> remove [px, py] from plan
        # if plan empty -> delete plan
        if self.has_plan():
            dp = np.linalg.norm(np.asarray([robot_x, robot_y])-np.asarray([self.current_goal.x, self.current_goal.y]))
            if dp <= 2:
                print("Captured poi")
                self.captured_goal = True

            dw = np.linalg.norm(np.array([robot_x, robot_y]) - self.current_plan[0])
            if dw <= 0.1:
                self.current_plan = np.delete(self.current_plan, [0], axis=0)
                print('removing completed waypoint')

                if len(self.current_plan) == 0:
                    print('removing completed plan')
                    self.delete_plan()
                    dh = np.linalg.norm(np.asarray([robot_x, robot_y]) - np.asarray([self.home.x, self.home.y]))
                    if dh <= 2:
                        print("Captured Home")
                        self.captured_home = True

        if self.has_plan():
            complete = False
        else:
            complete = True
        return complete


if __name__ == '__main__':
    from motion_planning.projections import Projector

    lat_center, lon_center = 40.01045433, 105.24432153
    projector = Projector(lat_center, lon_center)
    projector.setup()
    obs = [rrt.Obstacle(rrt.Obstacle.circle, (-5, 5), [2], 'ob')]
    m = MissionManager('../imgs/display_area.png', projector, obs, [], [])
    m.plan_to_from(0, 0, -10, 20, 0, 0)
    img = m.get_overlay_image_aspen(0, 0)
    img = Image.fromarray(img)
    img.show()
