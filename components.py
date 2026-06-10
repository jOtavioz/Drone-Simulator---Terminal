from constants import *
from env import ObstacleType
import random

class GPS():
    def __init__ (self):
        self._noiseStd = GPS_NOISE_STD
        self._x : float = 0.0
        self._y: float = 0.0
        self._z: float = 0.0
    
    def updatePosition(self, drone):
        self._x = drone.position[0] + random.gauss(0, self._noiseStd)
        self._y = drone.position[1] + random.gauss(0, self._noiseStd)
        self._z = drone.position[2] + random.gauss(0, self._noiseStd)
        
    @property
    def read(self) -> tuple[float, float, float]:
        return (self._x, self._y, self._z)
    


class LIDAR():
    def __init__(self):
        self._distance = 0
        self._maxRange = LIDAR_MAX_RANGE
        self._offset = LANDING_GEAR_HEIGHT
        self._noiseStd = LIDAR_NOISE_STD

    def detectObstacleBelow(self,drone,mission):
        x_drone = drone.position[0]
        y_drone = drone.position[1]
        for obstacle in mission.obstacles:
            if obstacle.type == ObstacleType.WALL:
             continue
            x_obs = obstacle.pos[0]
            y_obs = obstacle.pos[1]

            x_min = x_obs - obstacle.width / 2
            x_max = x_obs + obstacle.width / 2
            y_min = y_obs - obstacle.width / 2
            y_max = y_obs + obstacle.width / 2

            if (x_min <= x_drone <= x_max and y_min <= y_drone <= y_max):
                return (obstacle.pos[2] + obstacle.height)
        return 0
    
    def updateDistance(self, drone, mission):
        obstacle_height = self.detectObstacleBelow(drone, mission)
        self._distance = (drone.position[2] + self._offset - obstacle_height + random.gauss(0, self._noiseStd))
        self._distance = max (0, self._distance)
        self._distance = min (self._distance, self._maxRange)

    @property
    def read(self):
        return self._distance
    


class Motor():
    def __init__(self, name: str):
        self._name = name
        self._command = 0.0
        self._rpm: float = 0.0          # motor frequecy)
        self._thrust: float = 0.0       # Newton (N)
        
    def update(self, command):
        self._command = max (0, min(command, 1))    # 0 -> off; 1 -> max power
        self._rpm = self._command * RPM_MAX
        self._thrust = KF * (self._rpm ** 2)
        
    
    @property
    def thrust(self):
        return self._thrust

    @property
    def rpm(self):
        return self._rpm

    @property
    # returns in %
    def power(self):
        return (self._command*100)

    @property
    def name(self):
        return self._name


class MotorMixer():
    def __init__(self, motors):
        self.FL = motors[0]
        self.FR = motors[1]
        self.RL = motors[2]
        self.RR = motors[3]
    
    
    
    def mix_thrust(self, thrust_total, roll_cmd, pitch_cmd, yaw_cmd):
        t_base = thrust_total / 4.0
        thrusts = [
            t_base + roll_cmd + pitch_cmd - yaw_cmd,  # FL
            t_base - roll_cmd + pitch_cmd + yaw_cmd,  # FR
            t_base + roll_cmd - pitch_cmd + yaw_cmd,  # RL
            t_base - roll_cmd - pitch_cmd - yaw_cmd,  # RR
        ]
        T_MAX = KF * RPM_MAX**2
        motors = [self.FL, self.FR, self.RL, self.RR]
        for motor, t in zip(motors, thrusts):
            t = max(0.0, min(T_MAX, t))
            # converter thrust → command (0-1) normalizado
            rpm = math.sqrt(t / KF) if t > 0 else 0.0
            command = rpm / RPM_MAX
            motor.update(command)
                 










    def mix(self, throttle, roll, pitch, yaw):
        FL = throttle + roll + pitch - yaw
        FR = throttle - roll + pitch + yaw
        RL = throttle + roll - pitch + yaw
        RR = throttle - roll - pitch - yaw
        
        
        commands = [FL, FR, RL, RR]
        maxC = max(abs(c) for c in commands)
        if maxC > 1:
            commands = [ c / maxC for c in commands]
        commands = [ max(0,min(c,1)) for c in commands ]
        
        self.FL.update(commands[0])
        self.FR.update(commands[1])
        self.RL.update(commands[2])
        self.RR.update(commands[3])


