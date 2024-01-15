import argparse
import rospy

from motion_planning.waypoint_follower import WaypointFollower

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-s", "--settings",
                            help='path to settings file',
                            default="./settings.yaml")
        args = parser.parse_args()

        mission_area = WaypointFollower.GAZEBO
        robot = WaypointFollower.TARS
        wp = WaypointFollower(area=mission_area, robot=robot, settings=args.settings)
        wp.set_control(0.0)
        print('waiting for plan')
        rospy.spin()
        # wp.go_to_waypoints()
    except rospy.ROSInterruptException:
        pass
