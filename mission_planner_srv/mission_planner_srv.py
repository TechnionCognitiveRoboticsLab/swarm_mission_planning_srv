from swarm_interfaces.srv import MissionPlanningV2
from swarm_interfaces.msg import Mission, PlannedMissionV2, GeoPoint, Zone, ZoneTypeEnum, SubSwarmRoute
from map_cover.grid import *
import shapely
import numpy as np
from typing import List
import rclpy
from rclpy.node import Node


publish_plan_topic = "mission_plan"


class MissionPlanningService(Node):

    default_flight_altitude = 1.0

    def __init__(self):
        super().__init__('MissionPlanningService')
        self.srv = self.create_service(MissionPlanningV2, 'mission_planning', self.mission_planning_callback)  
        self.plan_publisher = self.create_publisher(PlannedMissionV2, publish_plan_topic, 10)  
        self.get_logger().info('Mission Planning Service Online')

    def mission_planning_callback(self, request, response):
        self.get_logger().info('Incoming request\na: %s' % str(request))
        mission = request.mission        
        cells = []
        for zone in mission.zones:
            area = shapely.Polygon([(p.longitude, p.latitude) for p in zone.geo_points])
            if zone.zone_type.value == ZoneTypeEnum.TARGET_DATA_ZONE:                
                dist = DiscreteProbabilityDistribution({zone.target_data.target_priority[i] : zone.target_data.target_detection_probability[i] for i in range(len(zone.target_data.target_priority))})
                cell = GridCellData(area, dist)
            elif zone.zone_type.value == ZoneTypeEnum.TARGET_DATA_ZONE:
                cell = GridCellData(area, DiscreteProbabilityDistribution({}), blocked=True)
            cells.append(cell)

        plan = []
        for cell in cells:
            center = cell.area.centroid
            plan.append(GeoPoint(longitude=center.x, latitude=center.y, altitude = self.default_flight_altitude))


        response.planned_mission = PlannedMissionV2()
        route = SubSwarmRoute()
        route.drones = mission.drones
        route.flight_altitude = self.default_flight_altitude
        route.route = plan
        response.planned_mission.routes.append(route)
        

        self.plan_publisher.publish(response.planned_mission)
        return response


def main(args=None):
    rclpy.init(args=args)

    planning_service = MissionPlanningService()

    rclpy.spin(planning_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
