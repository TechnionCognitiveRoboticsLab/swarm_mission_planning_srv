import rclpy
from rclpy.node import Node
import sys

from swarm_interfaces.srv import MapToGrid, MissionPlanningV2
from swarm_interfaces.msg import MapWithCoords, Zone, GeoPoint, ZoneShapeEnum, ZoneTypeEnum, MissionTargetData, Drone


def geopoint_from_xy(x, y):
    return GeoPoint(latitude = float(x), longitude = float(y), altitude = 0.0)

class MissionPlanningClientAsync(Node):
    def __init__(self):
        super().__init__('MissionPlanning_client')
        self.map_to_grid_cli = self.create_client(MapToGrid, 'map_to_grid')
        while not self.map_to_grid_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('map to grid service not available, waiting again...')
        self.map_to_grid_req = MapToGrid.Request()

        self.mission_planning_cli = self.create_client(MissionPlanningV2, 'mission_planning')
        while not self.mission_planning_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mission planning service not available, waiting again...')
        self.mission_planning_req = MissionPlanningV2.Request()

    def send_map_to_grid_request(self, filename):
        self.map_to_grid_req.input_map.map_filename = filename
        self.map_to_grid_req.input_map.bottom_left = geopoint_from_xy(0, 0)
        self.map_to_grid_req.input_map.top_right = geopoint_from_xy(100, 100)

        polygon1 = Zone()
        polygon1.geo_points = [geopoint_from_xy(5,5), geopoint_from_xy(15, 15), geopoint_from_xy(15, 5)]
        polygon1.target_data.target_priority = [1,5]
        polygon1.target_data.target_detection_probability = [0.8, 0.2]

        polygon2 = Zone()
        polygon2.geo_points = [geopoint_from_xy(60, 60), geopoint_from_xy(60, 90), geopoint_from_xy(90, 90), geopoint_from_xy(90, 60)]
        polygon2.target_data.target_priority = [1,5, 10]
        polygon2.target_data.target_detection_probability = [0.2, 0.3, 0.5]

        self.map_to_grid_req.additional_polygons = [polygon1, polygon2]

        self.map_to_grid_req.grid_cell_size = 10.0

        self.future_map_to_grid = self.map_to_grid_cli.call_async(self.map_to_grid_req)
        rclpy.spin_until_future_complete(self, self.future_map_to_grid)
        return self.future_map_to_grid.result()
    
    def send_mission_planning_request(self, zones):
        m = self.mission_planning_req.mission        
        d1 = Drone(drone_mission_duration=100, number_of_drone=1)
        d2 = Drone(drone_mission_duration=100, number_of_drone=2)
        d3 = Drone(drone_mission_duration=100, number_of_drone=3)
        m.drones = [d1, d2, d3]
        m.initial_location = [GeoPoint(longitude=0.0, latitude=0.0, altitude=0.0)] * len(m.drones)
        m.zones = zones
        
        self.future_mission_planning = self.mission_planning_cli.call_async(self.mission_planning_req)
        rclpy.spin_until_future_complete(self, self.future_mission_planning)
        return self.future_mission_planning.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MissionPlanningClientAsync()    
    response = minimal_client.send_map_to_grid_request(sys.argv[1])
    minimal_client.get_logger().info(
        'map to grid:  ' +  str(response))        
    
    plan = minimal_client.send_mission_planning_request(response.output_grid)
    minimal_client.get_logger().info(
        'plan:  ' +  str(plan))        

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()