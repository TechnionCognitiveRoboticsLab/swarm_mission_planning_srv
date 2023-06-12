from swarm_interfaces.srv import MissionPlanningV2
from swarm_interfaces.msg import Mission, PlannedMissionV2, GeoPoint, Zone, ZoneTypeEnum
import shapely
import numpy as np
from typing import List
import rclpy
from rclpy.node import Node


publish_plan_topic = "mission_plan"

class GridCellData:
    def __init__(self, area):
        self._target_data = {}        
        self._blocked = False
        self._area = area

    @property
    def blocked(self):
        return self._blocked
    
    @blocked.setter
    def blocked(self, value):
        self._blocked = value

    @property
    def target_data(self):
        return self._target_data

    def update_target_data(self, target_priority : List[int] = [], target_detection_probability : List[float] = []):
        for priority, probability in zip(target_priority, target_detection_probability):
            if priority in self._target_data:
                assert(self._target_data[priority] == probability)
            self._target_data[priority] = probability


    @property
    def area(self):
        return self._area

    def __repr__(self):
        return "area:" + str(self.area) + "__blocked:" + str(self.blocked) + "__target:" + str(self.target_data)


class MissionPlanningService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(MissionPlanningV2, 'mission_planning', self.mission_planning_callback)  
        self.plan_publisher = self.create_publisher(PlannedMissionV2, publish_plan_topic, 10)  


    def create_grid(self, delta=0.5, minx = 0, maxx = 10, miny = 0, maxy = 10):
        nx = int((maxx - minx)/delta)+1 
        ny = int((maxy - miny)/delta)+1 
        gx, gy = np.linspace(minx,maxx,nx), np.linspace(miny,maxy,ny)       
        grid = {}
        for i in range(len(gx)-1):
            for j in range(len(gy)-1):
                poly_ij = shapely.Polygon([[gx[i],gy[j]],[gx[i],gy[j+1]],[gx[i+1],gy[j+1]],[gx[i+1],gy[j]]])
                grid[i,j] = GridCellData(poly_ij)
        return grid


    def mission_planning_callback(self, request, response):
        mission = request.mission
        altitude = mission.initial_location.altitude + 1        
        grid = self.create_grid()
        for zone in mission.zones:
            polygon = shapely.Polygon( map(lambda geopoint: (geopoint.latitude, geopoint.longitude), zone.geo_points))            
            for i,j in grid:
                cell = grid[i,j]
                if polygon.intersects(cell.area):
                    if zone.zone_type.value == ZoneTypeEnum.FLIGHT_FORBIDDEN_ZONE:
                        cell.blocked = True
                    elif zone.zone_type.value == ZoneTypeEnum.TARGET_DATA_ZONE:                        
                        cell.update_target_data(zone.target_data.target_priority, zone.target_data.target_detection_probability)                        
        for x, y in grid:
            print(x, y, grid[x,y])

        plan = []


        response.planned_mission = PlannedMissionV2()
        self.get_logger().info('Incoming request\na: %s' % request)
        #response.planned_mission.drones = mission.drones
        response.planned_mission.flight_altitude = altitude
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
