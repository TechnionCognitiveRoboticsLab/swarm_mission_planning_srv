import rclpy
from rclpy.node import Node
import swarm_interfaces.msg
import swarm_interfaces.srv

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
        m.initial_location = [swarm_interfaces.msg.GeoPoint(longitude=0.0, latitude=0.0, altitude=0.0)] * len(m.drones)
        z = swarm_interfaces.msg.Zone ()    
        z.alt_max = 2
        z.alt_min = 1
        x = 4.0
        y = 3.0
        p1 = swarm_interfaces.msg.GeoPoint()
        p2 = swarm_interfaces.msg.GeoPoint()
        p3 = swarm_interfaces.msg.GeoPoint()
        p4 = swarm_interfaces.msg.GeoPoint()
        p1.altitude = 1.0
        p2.altitude = 1.0
        p3.altitude = 1.0
        p4.altitude = 1.0
        p2.latitude = x    
        p3.longitude = y
        p4.longitude = y
        p4.latitude = x

        z.geo_points = [p1, p2, p3, p4]
        m.zones = [z]

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
