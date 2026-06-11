from math import sin, cos, radians
from drone_sim.constants import *

'''
    Simplified Rigid Body Model

    p → roll angular velocity
    q → pitch angular velocity
    r → yaw angular velocity

    Ix,Iy,Iz → moments of inertia
    τx,τy,τz → torques

    roll += p·dt
    pitch += q·dt
    yaw += r·dt

    τ = I·α

    Convert thrust into linear acceleration:

    ax = (F/m)sin(pitch)

    ay = -(F/m)sin(roll)

    az = (F/m)cos(roll)cos(pitch)-g
'''

class Physics():

    @staticmethod
    def calculateTotalThrust(drone):
        total_thrust = sum(motor.thrust for motor in drone.motors)
        return total_thrust


    @staticmethod
    def calculateTorques(drone):

        FL = drone.motors[0].thrust
        FR = drone.motors[1].thrust
        RL = drone.motors[2].thrust
        RR = drone.motors[3].thrust

        roll_torque = ARM_LENGTH * (FL - FR + RL - RR)      # left side torques - right side torques

        pitch_torque = ARM_LENGTH * (FL + FR - RL - RR)     # front side torques - rear side torques

        return (roll_torque, pitch_torque)


    @staticmethod
    # convert torques em acc angulares -> accAng = torque/I
    def calculateAngularAcceleration(drone):
        roll_torque , pitch_torque = Physics.calculateTorques(drone)
        
        drone.angularAcceleration[0] = (roll_torque / IXX)
        drone.angularAcceleration[1] = (pitch_torque / IYY)


    @staticmethod
    # integra a aceleração angular para obter a velocidade angular
    def updateAngularVelocity(drone,dt):
        for i in range(2):
            drone.angularVelocity[i] += drone.angularAcceleration[i] * dt
            drone.angularVelocity[i] *= (1 - ANGULAR_DRAG * dt)

    @staticmethod
    # calculates the angle of inclination (Roll, Pitch) usando clamp para limitar
    def updateAttitude(drone,dt):                                       
        MAX_ANGLE = radians(10)
        for i in range(2):
            drone.attitude[i] += drone.angularVelocity[i] * dt
            drone.attitude[i] = max(-MAX_ANGLE, min(drone.attitude[i],MAX_ANGLE))


    @staticmethod
    def calculateLinearAcceleration(drone):
        total_thrust = Physics.calculateTotalThrust(drone)
        roll = drone.attitude[0]
        pitch = drone.attitude[1]

        ax = (total_thrust / MASS) * sin(pitch) - LINEAR_DRAG * drone.velocity[0]

        ay = (-total_thrust / MASS) * sin(roll) - LINEAR_DRAG * drone.velocity[1]

        az = (total_thrust / MASS) * cos(roll) * cos(pitch) - GRAVITY - LINEAR_DRAG * drone.velocity[2]

        drone.acceleration[0] = ax
        drone.acceleration[1] = ay
        drone.acceleration[2] = az
    
    @staticmethod
    # Euler Method --> integration of acceleration with respect to time 
    def updateVelocity(drone,dt):
        for i in range(3):
            drone.velocity[i] += drone.acceleration[i] * dt


    @staticmethod
    # Euler Method --> integration of velocity with respect to time 
    def updatePosition(drone,dt):
        for i in range(3):
            drone.position[i] += drone.velocity[i] * dt
        
        drone.position[2] = max(0,drone.position[2])


    @staticmethod
    def updatePhysics(drone,dt):
        Physics.calculateAngularAcceleration(drone)

        Physics.updateAngularVelocity(drone,dt)
        
        Physics.updateAttitude(drone,dt)

        Physics.calculateLinearAcceleration(drone)

        Physics.updateVelocity(drone,dt)

        Physics.updatePosition(drone,dt)
