import swarm_interfaces.msg
import swarm_interfaces.srv

import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(swarm_interfaces.srv.MissionPlanningV2, 'mission_planning')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = swarm_interfaces.srv.MissionPlanningV2.Request()

    def send_request(self):
        m = swarm_interfaces.msg.Mission()
        d1 = swarm_interfaces.msg.Drone(drone_mission_duration=100, number_of_drone=1)
        d2 = swarm_interfaces.msg.Drone(drone_mission_duration=100, number_of_drone=2)
        d3 = swarm_interfaces.msg.Drone(drone_mission_duration=100, number_of_drone=3)
        m.drones = [d1, d2, d3]
        m.initial_location.altitude = 0.0
        m.initial_location.latitude = 0.0
        m.initial_location.longitude = 0.0
        z = swarm_interfaces.msg.Zone ()
        z.alt_max = 2
        z.alt_min = 1
        

        zone1 = swarm_interfaces.msg.Zone()
        p1 = swarm_interfaces.msg.GeoPoint()
        p2 = swarm_interfaces.msg.GeoPoint()
        p3 = swarm_interfaces.msg.GeoPoint()        
        p1.altitude = 1.0
        p2.altitude = 1.0
        p3.altitude = 1.0        
        p1.latitude = 1.0   
        p1.longitude = 1.0
        p2.latitude = 1.0
        p2.longitude = 2.0
        p3.latitude = 2.0    
        p3.longitude = 2.0        
        zone1.geo_points = [p1, p2, p3]
        zone1.zone_type.value = swarm_interfaces.msg.ZoneTypeEnum.TARGET_DATA_ZONE
        zone1.target_data.target_priority = [1, 5]
        zone1.target_data.target_detection_probability = [0.8, 0.6]
        
        zone2 = swarm_interfaces.msg.Zone()
        p4 = swarm_interfaces.msg.GeoPoint()
        p5 = swarm_interfaces.msg.GeoPoint()
        p6 = swarm_interfaces.msg.GeoPoint()        
        p7 = swarm_interfaces.msg.GeoPoint()                
        p4.altitude = 1.0
        p5.altitude = 1.0
        p6.altitude = 1.0        
        p7.altitude = 1.0                
        p4.latitude = 3.0   
        p4.longitude = 3.0
        p5.latitude = 2.0
        p5.longitude = 4.0
        p6.latitude = 4.0    
        p6.longitude = 2.0        
        p7.latitude = 5.0    
        p7.longitude = 5.0        
        zone2.geo_points = [p4, p5, p6, p7]
        zone2.zone_type.value = swarm_interfaces.msg.ZoneTypeEnum.TARGET_DATA_ZONE
        zone1.target_data.target_priority = [5, 10]
        zone1.target_data.target_detection_probability = [0.4, 0.2]

        m.zones = [zone1, zone2]

        self.req.mission = m        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    print('Hi from mission_planner_client.')

    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info('got:  %s' % response)

    minimal_client.destroy_node()
    rclpy.shutdown()

    rclpy.init()




if __name__ == '__main__':
    main()
