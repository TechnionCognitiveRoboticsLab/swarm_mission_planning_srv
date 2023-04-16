from swarm_interfaces.srv import MissionPlanning
from swarm_interfaces.msg import Mission, PlannedMission

import rclpy
from rclpy.node import Node

 
class LawnmowerPlanningService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(MissionPlanning, 'mission_planning', self.mission_planning_callback)

    def mission_planning_callback(self, request, response):
        mission = request.mission
        min_x = mission.zones[0].geo_points[0].latitude
        min_y = mission.zones[0].geo_points[0].longitude
        max_x = mission.zones[0].geo_points[3].latitude
        max_y = mission.zones[0].geo_points[3].longitude

        response = PlannedMission()
        self.get_logger().info('Incoming request\na: %s' % request)

        return response


def main(args=None):
    rclpy.init(args=args)

    planning_service = LawnmowerPlanningService()

    rclpy.spin(planning_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()