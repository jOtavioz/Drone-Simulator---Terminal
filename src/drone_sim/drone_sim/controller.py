from drone_sim.constants import *


class PIDControl:
    def __init__(self, kp = PID_KP, ki = PID_KI, kd = PID_KD, integral_limit = PID_INTEGRAL_LIMIT, output_limit = PID_OUTPUT_LIMIT):
        
        self.kp=kp      # proportional gain --> multiplica o "erro atual" por um ganho
        self.ki=ki      # integral gain     --> corrige o erro acumulado de KP
        self.kd=kd      # derivate gain     --> antecipa as variações e evita overshoot ("freia com amortecimento")
        self.integral=0.0
        self.previous_error=0.0

        self.integral_limit = integral_limit
        self.output_limit = output_limit


    def calculateControl(self, error, dt):
        p_term = self.kp * error 

        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral,self.integral_limit))

        i_term = self.ki*self.integral
        
        derivative=0
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        d_term = (self.kd * derivative)
        
        self.previous_error = error

        output = p_term + i_term +d_term 

        return max(-self.output_limit, min(output, self.output_limit))


    def reset(self):
        self.integral = 0
        self.previous_error = 0
        

# ============================================================
# Waypoint controller
# ============================================================

class WaypointController():
    def __init__(self):
        # estágio 1: posição → velocidade alvo
        self.pidX_pos = PIDControl(kp=0.8,  ki=0.01, kd=0.1, output_limit=V_MAX)
        self.pidY_pos = PIDControl(kp=0.8,  ki=0.01, kd=0.1, output_limit=V_MAX)
        self.pidZ_pos = PIDControl(kp=0.8,  ki=0.01, kd=0.1, output_limit=V_MAX)

        # estágio 2: velocidade → aceleração
        self.pidX_vel = PIDControl(kp=2.0,  ki=0.05, kd=0.2, output_limit=A_MAX)
        self.pidY_vel = PIDControl(kp=2.0,  ki=0.05, kd=0.2, output_limit=A_MAX)
        self.pidZ_vel = PIDControl(kp=2.0,  ki=0.05, kd=0.2, output_limit=A_MAX)

    # Calcula os comandos de Roll, Pitch, Yaw
    # Obtem os erros de Posicao e Velocidade e os envia para os controladores PID
    # retorna o throttle, roll, pitch, yaw (adimensionais normalizados entre 0 e 1)
    def calculateCommands(self, drone, waypoint, dt):
        pos = drone.position
        vel = drone.velocity

        # estágio 1 — erro de posição → velocidade alvo
        vx = self.pidX_pos.calculateControl(waypoint.x - pos[0], dt)
        vy = self.pidY_pos.calculateControl(waypoint.y - pos[1], dt)
        vz = self.pidZ_pos.calculateControl(waypoint.z - pos[2], dt)

        # estágio 2 — erro de velocidade → aceleração
        ax = self.pidX_vel.calculateControl(vx - vel[0], dt)
        ay = self.pidY_vel.calculateControl(vy - vel[1], dt)
        az = self.pidZ_vel.calculateControl(vz - vel[2], dt)

        # thrust e atitude
        thrust = max(0.0, min(THRUST_MAX_TOTAL, MASS * (GRAVITY + az)))
        t_base = thrust / 4.0
        K_DRAG = 0.08

        pitch_ang = max(-0.61, min(0.61, ax / GRAVITY + math.atan(K_DRAG * vel[0]) / math.pi * 0.5))
        roll_ang  = max(-0.61, min(0.61, -ay / GRAVITY - math.atan(K_DRAG * vel[1]) / math.pi * 0.5))

        pitch_cmd = t_base * math.sin(pitch_ang)
        roll_cmd  = t_base * math.sin(roll_ang)

        return (thrust, roll_cmd, pitch_cmd, 0.0)

    def reset(self):
        for pid in [self.pidX_pos, self.pidY_pos, self.pidZ_pos,
                    self.pidX_vel, self.pidY_vel, self.pidZ_vel]:
            pid.reset()
