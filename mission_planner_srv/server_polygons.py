from swarm_interfaces.srv import MissionPlanningV2
from swarm_interfaces.msg import Mission, PlannedMissionV2, GeoPoint, Zone
import shapely
import numpy as np

import rclpy
from rclpy.node import Node


publish_plan_topic = "mission_plan"

 
class MissionPlanningService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(MissionPlanningV2, 'mission_planning', self.mission_planning_callback)  
        self.plan_publisher = self.create_publisher(PlannedMissionV2, publish_plan_topic, 10)  


    def create_grid(self, delta=0.5, minx = 0, maxx = 10, miny = 0, maxy = 10):
        nx = int((maxx - minx)/delta)+1 
        ny = int((maxy - miny)/delta)+1 
        gx, gy = np.linspace(minx,maxx,nx), np.linspace(miny,maxy,ny)
        print(nx, ny, gx, gy)
        grid = []
        for i in range(len(gx)-1):
            for j in range(len(gy)-1):
                poly_ij = shapely.Polygon([[gx[i],gy[j]],[gx[i],gy[j+1]],[gx[i+1],gy[j+1]],[gx[i+1],gy[j]]])
                grid.append( (i, j, poly_ij ) )
        return grid


    def mission_planning_callback(self, request, response):
        mission = request.mission
        altitude = mission.initial_location.altitude + 1        
        grid = self.create_grid()
        for zone in mission.zones:
            polygon = shapely.Polygon( map(lambda geopoint: (geopoint.latitude, geopoint.longitude), zone.geo_points))
            print(polygon)
            for i,j,cell in grid:
                if polygon.intersects(cell):
                    print(polygon, cell, i, j)
            


        plan = []


        response.planned_mission = PlannedMissionV2()
        self.get_logger().info('Incoming request\na: %s' % request)
        #response.planned_mission.drones = mission.drones
        response.planned_mission.flight_altitude = 1
        #response.planned_mission.initial_location = mission.initial_location
        #response.planned_mission.mission_type = mission.mission_type
        #response.planned_mission.target_data = mission.target_data
        #response.planned_mission.zones = [Zone(geo_points=plan)]
        response.planned_mission.route = plan

        self.plan_publisher.publish(response.planned_mission)
        return response


def main(args=None):
    rclpy.init(args=args)

    planning_service = MissionPlanningService()

    rclpy.spin(planning_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
