from swarm_interfaces.srv import MissionPlanningV2
from swarm_interfaces.msg import Mission, PlannedMissionV2, GeoPoint, Zone, ZoneTypeEnum, SubSwarmRoute
from map_cover.grid import *


from search import Instance, run, Agent
import shapely
import numpy as np
from typing import List
import rclpy
from rclpy.node import Node
import sys



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
        altitude = 1.0  # mission.initial_location.altitude + 1
        
        delta = mission.zones[0].geo_points[1].latitude - mission.zones[0].geo_points[0].latitude
        minx = mission.zones[0].geo_points[0].latitude
        maxx = mission.zones[-1].geo_points[1].latitude
        miny = mission.zones[0].geo_points[0].longitude
        maxy = mission.zones[-1].geo_points[1].longitude

        grid = self.create_grid(delta, minx, maxx, miny,maxy)
        for zone_i in range(len(mission.zones)):
            zone = mission.zones[zone_i]
            GPs = [(gp.latitude, gp.longitude) for gp in zone.geo_points]
            #print(f'Zone num {zone_i}: GPs: {GPs} Target Data: {zone.target_data.target_priority}: {zone.target_data.target_detection_probability}')

            polygon = shapely.Polygon( map(lambda geopoint: (geopoint.latitude, geopoint.longitude), zone.geo_points))
                        
            for i,j in grid:
                cell = grid[i,j]
                if shapely.intersection(cell.area, polygon).area > 0:
                    if zone.zone_type.value == ZoneTypeEnum.FLIGHT_FORBIDDEN_ZONE:
                        cell.blocked = True
                    elif zone.zone_type.value == ZoneTypeEnum.TARGET_DATA_ZONE:                        
                        cell.update_target_data(zone.target_data.target_priority, zone.target_data.target_detection_probability)
        
        width = max([xy[0] for xy in grid]) + 1
        length = max([xy[1] for xy in grid]) + 1
        blocked_cells = []
        

        agent_id=1
        agents = []
        num_unlocated_drones = len(mission.drones)
        
        # Distributing all of the drones as evenly as possible between init locations with default swarm size being 5:
        
        init_vertices = []
        for gp in mission.initial_location:
            loc_found=False
            for i,j in grid:
                cell = grid[i,j]
                p = shapely.Point(gp.latitude, gp.longitude)
                if cell.area.covers(p):
                    init_vertices.append(Instance.xy_to_id(i, j, width))
                    loc_found = True
                    break
            if not loc_found:
                raise Exception('Init Location outside of grid')

        HORIZON = max([drone.drone_mission_duration for drone in mission.drones])

        drones_per_loc = {}
        for v in init_vertices:
            if v not in drones_per_loc:
                drones_per_loc[v] = 1
            else:
                drones_per_loc[v]+=1
        for v in drones_per_loc:
            num_unlocated_drones = drones_per_loc[v]
            while num_unlocated_drones>0:
                subswarm_size = min(num_unlocated_drones, 5)
                agents.append(Agent.Agent(agent_id, v, HORIZON, subswarm_size))
                agent_id+=1
                num_unlocated_drones-=subswarm_size
                    

        
        ALGO = 'ASTAR' 
        
        '''
        Algorithms:

        ASTAR - one of the implemented algorithms similar to the A* algorithm. Other options include:
        BNBL - Branch and Bound with lower and upper heuristic.
        BNB - Branch and Bound with only upper heuristic.
        GBNB - Branch and bound with upper and lower heuristics algorithm that prioritizes short term reward over overall optimality.
        
        MCTS_V - monte carlo search that gives good results but unlike previous ones cant prove 
        that a solution is optimal therefore will stop at timeout and give best found solution untill that time.

        Empirically GBNB, ASTAR, MCTS_V work the best with their efficiency being roughly in that order.
        '''

        TIMEOUT = 180 # How much will the algorithm work before it stops and gives the best found solution


        for x, y in grid:
            if grid[x, y].blocked:
                blocked_cells.append((x, y))

        inst_map = Instance.gen_empty_grid(width, length, blocked_cells)
        inst = Instance.Instance('i1', inst_map, agents, HORIZON)
        inst.dropoffs=False


        for v in inst.map:
            x,y = Instance.id_to_xy(v.id, width)
            v.distribution = dict(grid[x, y].target_data)
            print(v.id, v.distribution)
        
        path = run.run_solver(inst, ALGO, TIMEOUT, return_path=True)
        print(path)
        mission_drones = mission.drones.copy()
        for agent in agents:
            sub_swarm_route = SubSwarmRoute()
            sub_swarm_drones = []
            route = []
            for _ in range(agent.utility_budget):
                drone = mission_drones[0]
                mission_drones.remove(drone)
                sub_swarm_drones.append(drone)

            for action in path[agent.id]:
                p = grid[Instance.id_to_xy(action.loc, width)].area.centroid
                gp = p.x, p.y
                gp = GeoPoint(longitude=p.x, latitude=p.y, altitude = altitude)
                route.append(gp)
            
            sub_swarm_route.route = route
            sub_swarm_route.drones = sub_swarm_drones
            sub_swarm_route.flight_altitude = altitude

            response.planned_mission.routes.append(route)

        self.get_logger().info('Incoming request\na: %s' % request)
        #response.planned_mission.flight_altitude = altitude
        self.plan_publisher.publish(response.planned_mission)
        return response


def main(args=None):
    rclpy.init(args=args)

    planning_service = MissionPlanningService()

    rclpy.spin(planning_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
