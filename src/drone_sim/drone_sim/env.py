from enum import Enum
from dataclasses import dataclass
from drone_sim.constants import *

class ObstacleType(Enum):
    PILLAR = 'pillar'   # obstacle that the drone must pass (go around) laterally
    BLOCK = 'block'     # obstacle that the drone must fly over
    WALL = 'wall'       # obstacle that indicates the end of mission

class Obstacle():
    def __init__(self, type: ObstacleType, x, y, z, height, width = None):
        self._pos = (x,y,z)
        self._height = height    # height of the object --> if z = 1 and height = 3, the obstacle ocupa z = 1 to z = 1 + 3
        self._width = width      # measured for the y to y + weight
        self._type = type
        
        if width is None:

            if type == ObstacleType.PILLAR:
                width = 0.5

            elif type == ObstacleType.BLOCK:
                width = 1.0

            elif type == ObstacleType.WALL:
                width = 20.0

        self._width = width
    
    @property
    def type(self):
        return self._type
    
    @property
    def pos(self) -> tuple:
        return self._pos
    
    @property
    def height(self):
        return self._height
    
    @property
    def width(self):
        return self._width

        
@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    description: str


class Mission():
    def __init__(self):
        self.obstacles = []
        self.waypoints = []
    
    
    def addWaypoint(self,waypoint:Waypoint):
        self.waypoints.append(waypoint)

    def addObstacle(self,obstacle:Obstacle):
        self.obstacles.append(obstacle)
        
    def printMission(self):

        print("\nEnvironment:\n")

        for obs in self.obstacles:

            print(f"{obs.type.value} "
                f"x = {obs.pos[0]} | "
                f"height = {obs.height} | "
                f"weight = {obs.width}"
            )

        print("\nRoute:\n")

        for i,wp in enumerate(self.waypoints):
            
            print(
                f"{i+1:2d} "
                f"{wp.description:<35}"
                f"({wp.x:.1f},"
                f"{wp.y:.1f},"
                f"{wp.z:.1f})"
            )



obs1 = Obstacle(ObstacleType.PILLAR, x=15,y=0,z=0,height=6)

obs2 = Obstacle(ObstacleType.BLOCK, x=30, y=0, z=0, height=3.0)

wall = Obstacle(ObstacleType.WALL, x=50, y=0, z=0, height=10)


waypoints = [
    Waypoint(0.0, 0.0, CRUISE_ALTITUDE, 'Initial takeoff to cruising altitude '),

    Waypoint(obs1.pos[0] - 2.0, 0.0, CRUISE_ALTITUDE ,'Approaching Obstacle 1'),

    Waypoint(obs1.pos[0], obs1.pos[1] + 2.0, CRUISE_ALTITUDE ,'Starting Lateral Avoidance'),

    Waypoint(obs1.pos[0] + 3.0, obs1.pos[1] + 2.0, CRUISE_ALTITUDE , 'Passing Around Pillar'),

    Waypoint(obs1.pos[0] + 5.0, 0.0, CRUISE_ALTITUDE , 'Returning to Main Route'),

    Waypoint(obs2.pos[0] - 2.0, 0.0, CRUISE_ALTITUDE , 'Approaching Obstacle 2'),

    Waypoint(obs2.pos[0], 0.0, obs2.height + 2.0, 'Climbing Above Obstacle 2'),

    Waypoint(obs2.pos[0] + 3.0, 0.0, obs2.height + 2.0, 'Passing Above Obstacle 2'),

    Waypoint(obs2.pos[0] + 5.0, 0.0, CRUISE_ALTITUDE, 'Returning to Cruise Altitude'),

    Waypoint(wall.pos[0] - 5.0, 0.0, CRUISE_ALTITUDE, 'Approaching Destination'),

    Waypoint(wall.pos[0] - 5.0, 0.0, 0.0, 'Landing')
]
