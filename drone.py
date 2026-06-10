
from components import (GPS, LIDAR, Motor, MotorMixer)


class Drone():
    def __init__ (self):
        self._position = [0.0, 0.0, 0.0]
        self._motors = [
            Motor("FL"),
            Motor("FR"),
            Motor("RL"),
            Motor("RR")
        ]
        self._velocity = [0.0,0.0,0.0]              # vx, vy, vz
        self._acceleration = [0.0,0.0,0.0]          # ax, ay, az
        
        self._attitude = [0.0,0.0,0.0]              # roll, pitch, yaw (rad)

        self._angularVelocity = [0.0,0.0,0.0]       # p,q,r (rad/s)

        self._angularAcceleration = [0.0,0.0,0.0]   # roll_acc,pitch_acc,yaw_acc (rad/s²)
        
        self._mixer = MotorMixer(self._motors)
        self._gps = GPS()
        self._lidar = LIDAR()
        
        
    @property
    def position(self):
        return self._position
    
    @property
    def velocity(self):
        return self._velocity

    @property
    def acceleration(self):
        return self._acceleration
    
    @property
    def attitude(self):
        return self._attitude


    @property
    def angularVelocity(self):
        return self._angularVelocity


    @property
    def angularAcceleration(self):
        return self._angularAcceleration
    
    
    @property
    def mixer(self):
        return self._mixer
    
    @property
    def gps(self):
        return self._gps
    
    @property
    def lidar(self):
        return self._lidar
    
    @property
    def motors(self):
        return self._motors
    

    def updateSensors(self,mission):
        self.gps.updatePosition(self)
        self.lidar.updateDistance(self, mission)