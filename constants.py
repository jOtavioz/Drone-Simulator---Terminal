import math
# ============================================================
# Simulation
# ============================================================

DT = 0.05                    # simulation step (s)
#SIMULATION_TIME = 60        # total simulation time (s)


# ============================================================
# Drone physical parameters
# ============================================================

MASS = 2.0                  # kg
GRAVITY = 9.81              # m/s²

ARM_LENGTH = 0.25           # motor-center distance (m)

LANDING_GEAR_HEIGHT = 0.15  # sensor height from ground (m)


# ============================================================
# Motor parameters
# ============================================================

RPM_MAX = 12000             # maximum motor rotation

KF = 3.25e-7                # thrust coefficient

KM = 7.5e-9                 # drag/yaw coefficient

V_MAX          = 2.0    # velocidade máxima por eixo (m/s)
A_MAX          = 5.0    # aceleração máxima por eixo (m/s²)

KP_POS         = 0.8    # ganho posição → velocidade

KP_VEL         = 3.0    # ganho velocidade → aceleração

THRUST_MAX_TOTAL = KF * RPM_MAX**2 * 4  # thrust máximo total (N)


# ============================================================
# GPS sensor
# ============================================================

GPS_NOISE_STD = 0.05        # ±5 cm

# ============================================================
# LiDAR sensor
# ============================================================

LIDAR_NOISE_STD = 0.01
LIDAR_MAX_RANGE = 8.0

# ============================================================
# Navigation
# ============================================================

CRUISE_ALTITUDE = 3.0
SAFE_OBSTACLE_MARGIN = 2.0

# ============================================================
# PID defaults
# ============================================================

PID_KP = 0.15
PID_KI = 0.02
PID_KD = 0.3

PID_INTEGRAL_LIMIT = 2.0
PID_OUTPUT_LIMIT = 0.35

HOVER_THROTTLE = math.sqrt(MASS * GRAVITY / (4 * KF * RPM_MAX**2))

# ============================================================
# Inertia
# ============================================================

IXX = 0.02
IYY = 0.02
IZZ = 0.04

KM = 7.5e-9


LINEAR_DRAG = 0.25

ANGULAR_DRAG = 0.1