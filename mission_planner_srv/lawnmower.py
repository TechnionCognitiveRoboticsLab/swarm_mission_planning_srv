from swarm_interfaces.srv import MissionPlanningV2
from swarm_interfaces.msg import Mission, PlannedMissionV2, GeoPoint, Zone, ZoneTypeEnum, SubSwarmRoute

import rclpy
from rclpy.node import Node


publish_plan_topic = "mission_plan"

 
class LawnmowerPlanningService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(MissionPlanningV2, 'mission_planning', self.mission_planning_callback)  
        self.plan_publisher = self.create_publisher(PlannedMissionV2, publish_plan_topic, 10)  

    def mission_planning_callback(self, request, response):
        self.get_logger().info('Incoming request\na: %s' % str(request))
        
        mission = request.mission
        plan = []

        plan.append(GeoPoint(altitude=0.0, latitude=300.0, longitude=280.0,))
        plan.append(GeoPoint(altitude=0.0, latitude=850.0, longitude=280.0,))
        plan.append(GeoPoint(altitude=0.0, latitude=300.0, longitude=280.0,))
        plan.append(GeoPoint(altitude=0.0, latitude=850.0, longitude=280.0,))

        response.planned_mission = PlannedMissionV2()
        route = SubSwarmRoute()
        route.drones = mission.drones
        route.flight_altitude = 1.0
        route.route = plan
        response.planned_mission.routes.append(route)
        
        self.plan_publisher.publish(response.planned_mission)
        return response

def main(args=None):
    rclpy.init(args=args)

    planning_service = LawnmowerPlanningService()

    rclpy.spin(planning_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
