import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D
from PIL import Image

from motion_planning import rrt
from motion_planning.projections import PointOfInterest


class MissionManager:
    ASPEN = 'aspen'
    OUTDOOR = 'outdoor'

    def __init__(self, mission_area_image_path, projector, pois, obstructions, hazards, power_draws):
        area_miny, area_maxy, area_minx, area_maxx = -2, 10, -2, 5
        self.display_bounds = [-3, 12, -3, 12]  # minx, maxx, miny, maxy
        self.projector = projector
        self.pois = {poi.name: poi for poi in pois}
        self.location = self.ASPEN
        self.current_plan = None
        self.ordered_goals = None
        self.all_goals = None
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
        if poi_string in self.pois:
            poi = self.pois[poi_string]
        else:
            self.delete_plan()
        if poi:
            if tofrom and poi_string != 'HOME':
                home = self.pois['H']
                self.plan_to_from(robot_x, robot_y, poi.x, poi.y, home.x, home.y)
                self.ordered_goals = [poi, home]
            else:
                self.plan_waypoints(robot_x, robot_y, poi.x, poi.y)
                self.ordered_goals = [poi]
        print('going to', self.ordered_goals)

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

    def get_overlay_image_aspen(self, robot_x, robot_y, robot_h, path_color='black'):
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
        for id, poi in self.pois.items():
            if id.upper() == 'H' or id.upper() == 'HOME':
                ax.scatter([poi.x], [poi.y], c='gold', s=200, marker='*')
                ax.annotate(poi.name, (poi.x + 4, poi.y), size='large', va='center', ha='center')
            else:
                ax.add_patch(Circle((poi.x, poi.y), radius=1, facecolor='green', edgecolor='black'))
                ax.annotate(poi.name, (poi.x, poi.y), size='large', va='center', ha='center')

        # plot the obstacles
        for o in self.obstructions:
            # Can't activate/fix/deactivate obstructions!
            rx, ry = o.center
            c = plt.Circle((rx, ry), radius=o.axis[0], edgecolor='red', facecolor='red', alpha=0.5)
            ax.add_patch(c)

        for o in self.hazards:
            if o.id in self.active_obstacles:
                rx, ry = o.center
                c = plt.Circle((rx, ry), radius=o.axis[0], edgecolor='orange', facecolor='orange', alpha=0.5)
                ax.add_patch(c)

        for o in self.power_draws:
            if o.id in self.active_obstacles:
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
        arrow = u'$\u2191$'
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(robot_h)
        plt.scatter([robot_x], [robot_y], marker=rotated_marker, s=600, facecolors='b', edgecolors='b', zorder=20)

        plt.ylabel('y (m)')
        plt.xlabel('x (m)')

        lines = [Line2D([], [], color="white", marker=u'$\u2191$', markersize=10, markeredgecolor='b', markerfacecolor="blue")]
        descriptions = ['Robot Position']

        if len(self.power_draws) > 0:
            line2 = Line2D([], [], color="white", marker='o', markersize=10, markerfacecolor="blue", alpha=0.5)
            lines.append(line2)
            descriptions.append('Battery draws')
        if len(self.obstructions) > 0:
            line3 = Line2D([], [], color="white", marker='o', markersize=10, markerfacecolor="red", alpha=0.5)
            lines.append(line3)
            descriptions.append('Blocked areas')
        if len(self.hazards) > 0:
            line4 = Line2D([], [], color="white", marker='o', markersize=10, markerfacecolor="orange", alpha=0.5)
            lines.append(line4)
            descriptions.append('Hazardous areas')

        plt.legend(lines, descriptions, numpoints=1, loc=1)
        #plt.grid()
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

    def get_overlay_image(self, robot_x, robot_y, robot_h, path_color='black'):
        if self.location == self.ASPEN:
            return self.get_overlay_image_aspen(robot_x, robot_y, robot_h, path_color)
        elif self.location == self.OUTDOOR:
            return self.get_overlay_image_outdoor(robot_x, robot_y, path_color)

    def update_progress(self, robot_x, robot_y):
        # if [rx, ry]-[px, py] < d -> remove [px, py] from plan
        # if plan empty -> delete plan
        # TODO fix this with new POI structure
        '''
        if self.has_plan():
            current_goal = self.ordered_goals[0]
            dp = np.linalg.norm(np.asarray([robot_x, robot_y])-np.asarray([current_goal.x, current_goal.y]))
            if dp <= 2:
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
                        self.captured_home = True
        '''
        if self.has_plan():

            dw = np.linalg.norm(np.array([robot_x, robot_y]) - self.current_plan[0])
            if dw <= 0.1:
                self.current_plan = np.delete(self.current_plan, [0], axis=0)
                print('removing completed waypoint')

            if len(self.current_plan) == 0:
                print('removing completed plan')
                self.delete_plan()

            if len(self.ordered_goals) > 0:
                current_goal = self.ordered_goals[0]
                dp = np.linalg.norm(np.asarray([robot_x, robot_y]) - np.asarray([current_goal.x, current_goal.y]))
                if dp <= 2:
                    self.ordered_goals.remove(current_goal)
                    if 'H' in current_goal.name:
                        self.captured_home = True
                        print('captured home')
                    else:
                        self.captured_goal = True
                        print('captured goal')

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
    pois = [PointOfInterest(10, 10, name='H'), PointOfInterest(-2, -10, name='A')]
    m = MissionManager('../imgs/display_area.png', projector, pois, obs, [], [])
    m.plan_to_from(0, 0, pois[0].x, pois[0].y, pois[1].x, pois[1].y)
    img = m.get_overlay_image_aspen(0, 0, 45)
    img = Image.fromarray(img)
    img.show()
